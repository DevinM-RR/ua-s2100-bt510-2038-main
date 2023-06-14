/******************************************************************************
 * @file led.c
 * 
 * @brief LED blink pattern support using timers and a work queue to offload
 * sequencing from the main application. Supports hardware with a bi-color
 * LED configured on the board as led0 (green) and led1 (red).
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/
#include <zephyr.h>
#include <string.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <stdlib.h>

#include <shell/shell.h>
#include <shell/shell_uart.h>

/* register this module for logging */
#define LOG_LEVEL LOG_LEVEL_DBG
#define LOG_MODULE_NAME led
#include <logging/log.h>
LOG_MODULE_REGISTER(led);

#include "led.h"

/* durations for long, short and continuous LED patterns */
#define LED_ON_SHORT_MS 25
#define LED_ON_LONG_MS 1000
#define LED_OFF_SHORT_MS 250
#define LED_OFF_LONG_MS 1000
#define LED_CONTINUOUS_PERIOD_MS 1000
#define LED_DFU_CONTINUOUS_PERIOD_MS 500


/* module global device handle for GPIO access */
static const struct device *dev;

/*
 * LED pattern state structure
 */
struct led_info {
    int count; /* sequence counter for stepping through LED pattern operations in sequence */
    enum LED_PATTERN pattern;
} led_state;

/*
 * LED operation defines a duration to set the timer for, 
 * LED index and desired operation to perform on the LED 
 * until duration expires
 */
struct led_op {
    uint8_t duration;  // 16ms * duration = time to next operation. If 0, execute the next operation right away
    uint8_t led : 3;   // LED index to operate on
    uint8_t op : 5;    // operation on the LED
};

/*
 * Macro to convert milliseconds to the "duration" parameter used in the led_op structure
 */
#define LED_MS_TO_DURATION(ms) ((ms >> 4) & 0xFF)
#define LED_DURATION_TO_MS(d) (d << 4)

/*
 * LED sequence structure, stores a sequence of LED operations to define a pattern
 */
#define LED_OPERATIONS_MAX_COUNT 16
struct led_sequence {
    struct led_op op_sequence[LED_OPERATIONS_MAX_COUNT];
};

/*
 * LED timer, used to sequence through LED pattern operations
 */
static void led_timer_handler(struct k_timer *dummy);
K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);

/*
 * Is an LED sequence currently running
 */
static bool led_sequence_running = false;

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#if DT_PHA_HAS_CELL(LED0_NODE, gpios, flags)
#define LED0_FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#endif
#else
/* A build error here means your board does not support LED0 */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define LED0_PIN	0
#endif

#ifndef LED0_FLAGS
#define LED0_FLAGS	0
#endif

/* The devicetree node identifier for the "led1" alias. */
#define LED1_NODE DT_ALIAS(led1)

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
#define LED1	DT_GPIO_LABEL(LED1_NODE, gpios)
#define LED1_PIN	DT_GPIO_PIN(LED1_NODE, gpios)
#if DT_PHA_HAS_CELL(LED1_NODE, gpios, flags)
#define LED1_FLAGS	DT_GPIO_FLAGS(LED1_NODE, gpios)
#endif
#else
/* A build error here means your boarddoes not support LED1 */
#error "Unsupported board: led1 devicetree alias is not defined"
#define LED1	""
#define LED1_PIN	0
#endif

#ifndef LED1_FLAGS
#define LED1_FLAGS	0
#endif

/*
 * LED pattern sequence definitions
 */
struct led_sequence led_sequences[] = {
    // LED_PATTERN_OFF - both LEDs off
    {
        .op_sequence = {
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_RED, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    },
    // LED_PATTERN_ON_OK - short blink green 1 time
    {
        .op_sequence = {
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    },
    // LED_PATTERN_PRESS_2_SEC - short blink red 1 time
    {
        .op_sequence = {
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_RED, .op = LED_OP_ON },
            { .duration = 0, .led = LED_RED, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    },
    // LED_PATTERN_HEARTBEAT - short blink green 1 time
    {
        .op_sequence = {
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    },
    // LED_PATTERN_PROVISIONING_MODE - short blink green + red continuous
    {
        .op_sequence = {
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_RED, .op = LED_OP_ON },
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = LED_MS_TO_DURATION(LED_CONTINUOUS_PERIOD_MS), .led = LED_RED, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_LOOP }
        }
    },
    // LED_PATTERN_PROVISION_COMPLETE - long blink green 3 times
    {
        .op_sequence = {
            { .duration = LED_MS_TO_DURATION(LED_ON_LONG_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_OFF_LONG_MS), .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = LED_MS_TO_DURATION(LED_ON_LONG_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_OFF_LONG_MS), .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = LED_MS_TO_DURATION(LED_ON_LONG_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    },
    // LED_PATTERN_CONNECTED - short blink green 2 times
    {
        .op_sequence = {
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    },
    // LED_PATTERN_DFU_COMPLETE - long blink green + red
    {
        .op_sequence = {
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_ON_LONG_MS), .led = LED_RED, .op = LED_OP_ON },
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_RED, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    },
    // LED_PATTERN_POWER_ON - long blink green
    {
        .op_sequence = {
            { .duration = LED_MS_TO_DURATION(LED_ON_LONG_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    },
    // LED_PATTERN_LOW_BATT - short blink red 3 times
    {
        .op_sequence = {
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_RED, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_OFF_SHORT_MS), .led = LED_RED, .op = LED_OP_OFF },
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_RED, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_OFF_SHORT_MS), .led = LED_RED, .op = LED_OP_OFF },
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_RED, .op = LED_OP_ON },
            { .duration = 0, .led = LED_RED, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    },
    // LED_PATTERN_DFU_IN_PROGRESS - short blink green fast continuous
    {
        .op_sequence = {
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_DFU_CONTINUOUS_PERIOD_MS), .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_RED, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_DFU_CONTINUOUS_PERIOD_MS), .led = LED_RED, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_LOOP }
        }
    },
    // LED_PATTERN_FACTORY_RESET - long blink red 1 time
    {
        .op_sequence = {
            { .duration = LED_MS_TO_DURATION(LED_ON_LONG_MS), .led = LED_RED, .op = LED_OP_ON },
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_RED, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    },
    // LED_PATTERN_DATALOG_FULL - short blink green 3 times
    {
        .op_sequence = {
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_OFF_SHORT_MS), .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = LED_MS_TO_DURATION(LED_OFF_SHORT_MS), .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = LED_MS_TO_DURATION(LED_ON_SHORT_MS), .led = LED_GREEN, .op = LED_OP_ON },
            { .duration = 0, .led = LED_GREEN, .op = LED_OP_OFF },
            { .duration = 0, .led = LED_NONE, .op = LED_OP_END }
        }
    }
};
#define LED_SEQUENCES_COUNT (sizeof(led_sequences) / sizeof(struct led_sequence))

/*
 * Turn off an LED
 */
static void led_off(int idx)
{
    switch(idx) {
        case LED_GREEN:
            gpio_pin_set(dev, LED0_PIN, 0);
            break;
        case LED_RED:
            gpio_pin_set(dev, LED1_PIN, 0);
            break;
        default:
            break;
    }
}

/*
 * Turn on an LED
 */
static void led_on(int idx)
{
    switch(idx) {
        case LED_GREEN:
            gpio_pin_set(dev, LED0_PIN, 1);
            break;
        case LED_RED:
            gpio_pin_set(dev, LED1_PIN, 1);
            break;
        default:
            break;
    }
}

/*
 * Perform the next operation in the selected LED pattern
 */
static void led_process_op()
{
    // If an invalid pattern, just return
    if(led_state.pattern >= LED_SEQUENCES_COUNT) {
        return;
    }

    // Process the next operation in the 'pattern' at entry 'led_state.count'
    struct led_sequence *seq = &led_sequences[led_state.pattern];

    // Check that the sequence count/index is within bounds
    if(led_state.count >= LED_OPERATIONS_MAX_COUNT) {
        return;
    }

    /* Loop through operations until duration > 0 or we hit LED_OP_END */
    do {
        struct led_op *op = &seq->op_sequence[led_state.count];
        led_state.count++;

        switch(op->op) {
            case LED_OP_END:
                // Do nothing, end the sequence
                break;
            case LED_OP_OFF:
                // Turn off the LED
                led_off(op->led);
                break;
            case LED_OP_ON:
                // Turn on the LED
                led_on(op->led);
                break;
            case LED_OP_LOOP:
                // Reset the led_state.count to 0
                led_state.count = 0;
                continue;
                break;
            case LED_OP_RAMP_ON:
                // Ramp an LED up via PWM
                // TODO
                break;
            case LED_OP_RAMP_OFF:
                // RAMP an LED down via PWM
                // TODO
                break;
            default:
                break;
        }
        
        /* If duration is non-zero, start a timer to process the next operation */
        if(op->duration > 0) {
            k_timer_start(&led_timer, K_MSEC(LED_DURATION_TO_MS(op->duration)), K_MSEC(0));
            break;
        }

        /* If this is an LED_OP_END op, stop processing */
        if(op->op == LED_OP_END) {
            led_sequence_running = false;
            break;
        }

    } while(led_state.count < LED_OPERATIONS_MAX_COUNT);
}

/*
 * Starts an LED sequence in caller's context, uses led_timer
 * to step through desired sequence of LED on/off patterns
 */
static void led_pattern_start(void)
{
    /* stop the periodic led timer */
    k_timer_stop(&led_timer);

    /* turn off LEDs */
    led_off(LED_GREEN);
    led_off(LED_RED);

    /* restart the LED pattern sequence counter */
    led_state.count = 0;
    led_sequence_running = true;

    /* process the first operation in the sequence */
    led_process_op();
}

/*
 * Change LED states and reset the LED timer
 */
static void led_timer_handler(struct k_timer *dummy)
{
    led_process_op();
}

/*
 * Shell LED pattern test command
 */

static int cmd_led_pattern(const struct shell *shell, size_t argc, char **argv)
{
	if(argc < 2) {
        LOG_WRN("must specify pattern (0-%d)", (LED_SEQUENCES_COUNT) - 1);
    } else {
        uint32_t led_pattern = atoi(argv[1]);
        if (led_pattern < LED_PATTERN_COUNT)
        {
            LOG_DBG("LED pattern set to %d", led_pattern);
            led_trigger_pattern((enum LED_PATTERN)led_pattern);
        }
        else
        {
            LOG_WRN("Invalid LED pattern %d", led_pattern);
        }
    }

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_led,
	SHELL_CMD(pattern, NULL, "LED pattern command.", cmd_led_pattern),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(led, &sub_led, "LED commands", NULL);

/*****************************************************************************
 * PUBLIC API
 ****************************************************************************/

/*
 * Initialize the LED pattern handling resources,
 * call once on startup.
 */
void led_init(void)
{
    int ret;

	dev = device_get_binding(LED0);
	if (dev == NULL) {        
        LOG_ERR("Leds initialized failed.");
		return;
	}


	ret = gpio_pin_configure(dev, LED0_PIN, GPIO_OUTPUT_INACTIVE | LED0_FLAGS);
	if (ret < 0) {
        LOG_ERR("LED0 configuration error %d", ret);
        return;
	}

	ret = gpio_pin_configure(dev, LED1_PIN, GPIO_OUTPUT_INACTIVE | LED1_FLAGS);
	if (ret < 0) {
        LOG_ERR("LED1 configuration error %d", ret);
		return;
	}
}

/*
 * Trigger a pattern on the LEDs. If a pattern is currently executing,
 * subsequent calls will immediately start the desired pattern,
 * interrupting the previous pattern.
 */
void led_trigger_pattern(enum LED_PATTERN pattern)
{
    /* stop the periodic led timer */
    k_timer_stop(&led_timer);

    /* set the selected LED pattern */
    led_state.pattern = pattern;
    led_pattern_start();
}

/*
 * Get the current LED pattern
 */
enum LED_PATTERN led_get_pattern(void)
{
    return led_state.pattern;
}

/*
 * Return true if LED sequence is running, false if not
 */
bool led_sequence_is_running(void)
{
    return led_sequence_running;
}
