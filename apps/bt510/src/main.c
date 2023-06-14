/******************************************************************************
 * @file main.c
 * 
 * @brief main entry point for contact tracing application
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/

#include <zephyr.h>
#include <stats/stats.h>
#include <power/power.h>
#include <power/reboot.h>
#include <device.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <dfu/mcuboot.h>
#include <power/reboot.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <stdlib.h>
#include <crypto/cipher.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#define LOG_MODULE_NAME main
#include <logging/log.h>
#include <logging/log_ctrl.h>
LOG_MODULE_REGISTER(main);

#include "storage.h"
#include "smp.h"
#include "led.h"
#include "button.h"
#include "systime.h"
#include "accel.h"
#include "battery.h"
#include "led.h"
#include "ct_qrtc.h"
#include "DataLog.h"
#include "Settings.h"
#include "temperature.h"

#define CONSOLE_LABEL DT_LABEL(DT_CHOSEN(zephyr_console))

/* device handle for the console UART */
static const struct device *cons;

#if defined(CONFIG_BOARD_BT510)

#define TM_INPUT_NODE DT_ALIAS(tminput)
#if DT_NODE_HAS_STATUS(TM_INPUT_NODE, okay)
#define TM_INPUT_GPIO_LABEL DT_GPIO_LABEL(TM_INPUT_NODE, gpios)
#define TM_INPUT_GPIO_PIN DT_GPIO_PIN(TM_INPUT_NODE, gpios)
#define TM_INPUT_GPIO_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(TM_INPUT_NODE, gpios))
#else
#error "Unsupported board: tminput devicetree alias is not defined"
#define TM_INPUT_GPIO_LABEL ""
#define TM_INPUT_GPIO_PIN 0
#define TM_INPUT_GPIO_FLAGS 0
#endif

#elif defined(CONFIG_BOARD_BL654_DVK)

#define TM_INPUT_NODE DT_ALIAS(sw1)
#if DT_NODE_HAS_STATUS(TM_INPUT_NODE, okay)
#define TM_INPUT_GPIO_LABEL DT_GPIO_LABEL(TM_INPUT_NODE, gpios)
#define TM_INPUT_GPIO_PIN DT_GPIO_PIN(TM_INPUT_NODE, gpios)
#define TM_INPUT_GPIO_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(TM_INPUT_NODE, gpios))
#else
#error "Unsupported board: tminput devicetree alias is not defined for BL654_dvk"
#define TM_INPUT_GPIO_LABEL ""
#define TM_INPUT_GPIO_PIN 0
#define TM_INPUT_GPIO_FLAGS 0
#endif

#endif

static uint32_t reset_reason = 0;

/* set stack size and priority for this module's work q thread */
#define MAIN_WQ_STACK_SIZE 4096
#define MAIN_WQ_PRIORITY 7

/* declare this module's work q resources */
K_THREAD_STACK_DEFINE(main_wq_stack_area, MAIN_WQ_STACK_SIZE);
static struct k_work_q main_work_q;

static struct k_work state_change_boot_work;
static struct k_work state_change_on_sleep_work;
static struct k_work state_change_factory_reset_work;

struct cliSetTime_work_t
{
    struct k_work work;
    uint32_t time;
    int32_t resynchTestCnt;
    uint32_t resynchTestTimerPeriod;

} cliSetTime_work;


/* store the mcuboot swap_type variable */
static int swap_type = BOOT_SWAP_TYPE_NONE;

/* keep track of first entry into "on sleep" state */
static bool first_on = true;

/* keep track of whether we are in low power mode */
static int power_state = 1;

/* Test mode pin value read on startup. Default to high to indicate normal operation */
#define TEST_MODE_ENABLED 0
static uint8_t tmVal = !TEST_MODE_ENABLED;

static void nrf_nvmc_write_word(uint32_t address, uint32_t value);
void state_on_sleep(struct k_work *work);
static void print_thread_cb(const struct k_thread *thread, void *user_data);

enum APP_STATE {
    APP_STATE_BOOT,
    APP_STATE_ON_SLEEP,
};

static enum APP_STATE app_state = APP_STATE_BOOT;

/*
 * timer to detect whether button is held down 20 seconds, in which
 * case the red LED should blink indicating a factory reset will occur
 * when the button is released.
 */
// static void factory_reset_timer_handler(struct k_timer *dummy);
// K_TIMER_DEFINE(factory_reset_timer, factory_reset_timer_handler, NULL);

static void rtc_resynch_test_timer_handler(struct k_timer *dummy);
K_TIMER_DEFINE(rtc_resynch_test_timer, rtc_resynch_test_timer_handler, NULL);

/*
 * trigger "FACTORY_RESET" LED pattern
 */
// static void factory_reset_timer_handler(struct k_timer *dummy)
// {
//     // led_trigger_pattern(LED_PATTERN_FACTORY_RESET);
// }

static void rtc_resynch_test_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&cliSetTime_work.work);
}

/*
 * helper function to submit a work object to change state
 */
static void change_state(struct k_work *new_state)
{
    k_work_submit_to_queue(&main_work_q, new_state);
}

/*
 * button changed handler for gpio pin interrupt
 */
// static void button_changed(struct device *dev, struct gpio_callback *cb, uint32_t pins)
// {
//     static uint64_t press_time = 0;
//     uint64_t hold_ms = 0;

//     int button_state = button_read();

// 	if(button_state) {
//         press_time = k_uptime_get();

// 		switch(app_state) {

//         case APP_STATE_ON_SLEEP:
//             // k_timer_start(&factory_reset_timer, K_MSEC(10000), K_MSEC(0));
//             break;

//         default:
//             break;
//         }
// 	} else {
//         if(press_time != 0) /* make sure the button was pressed by this handler first before processing a release */
//         {
//             hold_ms = k_uptime_delta(&press_time);

//             switch(app_state) {

//             case APP_STATE_ON_SLEEP:
//                 // k_timer_stop(&factory_reset_timer);

//                 // if(hold_ms > 10000) {
//                 //     // submit work items to stop bluetooth operations during a factory reset
//                 //     stop_smp_bluetooth();
//                 //     change_state(&state_change_factory_reset_work);
//                 // } 
//                 break;

//             default:
//                 break;
//             }
//         }
//     }
// }

static void nrf_nvmc_write_word(uint32_t address, uint32_t value)
{
    // Enable write.
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    __ISB();
    __DSB();

    *(uint32_t*)address = value;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {;}

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    __ISB();
    __DSB();
}

void enter_deep_sleep(void)
{
    led_trigger_pattern(LED_PATTERN_OFF);

    /* init the primary button to wake from deep sleep*/
    //button_init_gpiote(button_changed_gpiote);

    nrf_gpiote_event_clear(NRF_GPIOTE, NRF_GPIOTE_EVENT_PORT);
    pm_ctrl_enable_state(POWER_STATE_DEEP_SLEEP_1);
    k_sleep(K_MSEC(1));

    /* on button press, the device will reset with
     * reset reason POWER_RESETREAS_OFF_Msk bit set */
}

//called by nrf_power_clock.c
void nrfx_power_pof_event_callback(void)
{
    //Reset into shipment mode
    power_state = 0;
    app_state = APP_STATE_BOOT;

    LOG_ERR("Resetting system due to POF");
    LOG_PANIC();
    k_sleep(K_MSEC(1000));
    sys_reboot(SYS_REBOOT_COLD);
}

/*
 * One-time initialization at first boot
 */
void state_on_sleep_first_on(void)
{
    smp_read_image_header(&img_header);//use global struct

    /* mount the /lfs partition so it can be used by settings */
    storage_init();

    SettingsInit();
    
    RtcInit();

    DataLogInit();

    TMP_Init();

    /* using __TIME__ ensure that a new binary will be built on every
	* compile which is convient when testing firmware upgrade.
	*/
    LOG_INF("Version %d.%d.%d %s %s", img_header.h.v1.sem_ver.major, img_header.h.v1.sem_ver.minor, img_header.h.v1.sem_ver.revision, __DATE__, __TIME__);

    first_on = false;
}

/*
 * "on sleep" state, primary state for the tracking application
 */
void state_on_sleep(struct k_work *work)
{
    /* No longer in low power mode */
    power_state = 1;

    app_state = APP_STATE_ON_SLEEP;
    LOG_INF("[APP_STATE_ON_SLEEP]");
    LOG_INF("swap_type: %d", swap_type);

    k_sleep(K_MSEC(1000));//delay to print

    smp_start();

    accel_init();

}

void ProtectJtag()
{
    if (NRF_UICR->APPROTECT == 0xFFFFFFFF)
    {
        nrf_nvmc_write_word((uint32_t)&(NRF_UICR->APPROTECT), 0xFFFFFF00);
        
        k_sleep(K_MSEC(1000));
        NVIC_SystemReset();
    }
    else
    {
        // LOG_DBG("JTAG disabled");
    }
}

/*
 * one-time init at boot to initialize hardware
 */
void state_boot(struct k_work *work)
{
    led_init();

    reset_reason = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear reset reason by writing 1.
    
    /* Get device binding for console uart for device power management */
    cons = device_get_binding(CONSOLE_LABEL);
    if (cons == NULL)
    {
        LOG_ERR("UART not found\n");
    }
    else
    {
        /* Read the TM pin on startup to decide whether to keep UART enabled or not */
        static const struct device *tmInput;
        tmInput = device_get_binding(TM_INPUT_GPIO_LABEL);
        if (tmInput == NULL)
        {
            LOG_ERR("Error: didn't find %s device\n", TM_INPUT_GPIO_LABEL);
        }
        int32_t ret = gpio_pin_configure(tmInput, TM_INPUT_GPIO_PIN, TM_INPUT_GPIO_FLAGS);
        if (ret != 0)
        {
            LOG_ERR("Error %d: failed to configure %s pin %d\n",
                    ret, TM_INPUT_GPIO_LABEL, TM_INPUT_GPIO_PIN);
        }

        k_sleep(K_MSEC(10));
        tmVal = gpio_pin_get(tmInput, TM_INPUT_GPIO_PIN);

        ret = gpio_pin_configure(tmInput, TM_INPUT_GPIO_PIN, 0);
        if (ret != 0)
        {
            LOG_ERR("Error %d: failed to configure %s pin %d to disconnected\n",
                    ret, TM_INPUT_GPIO_LABEL, TM_INPUT_GPIO_PIN);
        }

        if (tmVal == TEST_MODE_ENABLED)
        {
            device_set_power_state(cons, DEVICE_PM_ACTIVE_STATE, NULL, NULL);
        }
        else
        {
            device_set_power_state(cons, DEVICE_PM_LOW_POWER_STATE, NULL, NULL);
            LOG_DBG("Console UART disabled");
        }
    }

    LOG_DBG("reset reason: 0x%x", reset_reason);

    if (first_on)
    {
        state_on_sleep_first_on();
    }

    /* initialize smp for device management */
    swap_type = smp_init();

    //button_init(button_changed);//added 13 uA

    change_state(&state_change_on_sleep_work);
}

void k_sys_fatal_error_handler(unsigned int reason,
                               const z_arch_esf_t *esf)
{
    ARG_UNUSED(esf);

    LOG_ERR("Resetting system due to %d", reason);
    LOG_PANIC();
    k_sleep(K_MSEC(1000));
    sys_reboot(SYS_REBOOT_COLD);
}

/*
 * factory reset state - clear the data logs and reboot, entering off_sleep state
 */
void state_factory_reset(struct k_work *work)
{
    LOG_INF("[APP_STATE_FACTORY_RESET]");

    /* make sure to delay until the factory reset LED pattern is completed */
    k_sleep(K_MSEC(1000));

    /* force LEDs off just in case */
    led_trigger_pattern(LED_PATTERN_OFF);

    LOG_INF("REBOOTING...");
    k_sleep(K_MSEC(250));
    sys_reboot(SYS_REBOOT_COLD);
}

void cliSetTime_work_handler(struct k_work *work)
{
    struct cliSetTime_work_t *params =
        CONTAINER_OF(work, struct cliSetTime_work_t, work);

    if (params->resynchTestCnt == 0)
    {
        if (k_timer_remaining_ticks(&rtc_resynch_test_timer) > 0)
        {
            LOG_INF("RTC resynch test stopped");
        }
        k_timer_stop(&rtc_resynch_test_timer);

        SysTime_t newTime;
        newTime.Seconds = params->time;
        newTime.SubSeconds = 0;

        bool success = SysTimeSet(newTime);

        if (success)
        {
            set_time_src(TIME_SRC_BLE_CLI);
        }
        else
        {
            LOG_INF("Invalid time - %d", params->time);
        }
    }
    else
    {
        SysTime_t newTime;
        newTime = SysTimeGet();
        newTime.Seconds += params->resynchTestCnt;
        bool success = SysTimeSet(newTime);

        if (success)
        {
            set_time_src(TIME_SRC_BLE_CLI);
            k_timer_stop(&rtc_resynch_test_timer);
            if (params->resynchTestTimerPeriod > 0)
            {
                k_timer_start(&rtc_resynch_test_timer, K_MSEC(params->resynchTestTimerPeriod * 1000), K_MSEC(0));
                LOG_INF("RTC resynch test started (val-%d, period-%d)", params->resynchTestCnt, params->resynchTestTimerPeriod);
            }
        }
        else
        {
            LOG_INF("Invalid time - %d", params->time);
            k_timer_stop(&rtc_resynch_test_timer);
        }
    }
}

/*
 * init work q and work items for processing state changes
 */
void init_work(void)
{
    k_work_init(&state_change_boot_work, state_boot);
    k_work_init(&state_change_on_sleep_work, state_on_sleep);
    k_work_init(&state_change_factory_reset_work, state_factory_reset);
    k_work_init(&cliSetTime_work.work, cliSetTime_work_handler);
    k_work_q_start(&main_work_q, main_wq_stack_area,
                   K_THREAD_STACK_SIZEOF(main_wq_stack_area), MAIN_WQ_PRIORITY);
}

/*
 * application entry point
 */
void main(void)
{
    /* Prevent deep sleep (system off) from being entered on long
	 * timeouts due to the default residency policy.
	 */
    pm_ctrl_disable_state(POWER_STATE_DEEP_SLEEP_1);

    /* init the work q and work structs */
    init_work();

    /* change to "boot" state */
    change_state(&state_change_boot_work);

    /* sleep forever */
	while (1) {
        k_sleep(K_MSEC(15000));
    }
}

/* Requires CONFIG_THREAD_MONITOR and CONFIG_THREAD_NAME */
void print_thread_list(void)
{
    uint32_t thread_count = 0;
    k_thread_foreach(print_thread_cb, &thread_count);
    printk("Preemption is %s\r\n",
           (CONFIG_PREEMPT_ENABLED) ? "Enabled" : "Disabled");
}

static uint32_t calc_used_stack(uint8_t *stack_start, uint32_t stack_size)
{
	uint32_t used = 0;
	uint8_t *ptr = stack_start;

	while(ptr[0] == 0xaa && stack_size) {
		stack_size--;
		used++;
		ptr++;
	}
	return used;
}

static void print_thread_cb(const struct k_thread *thread, void *user_data)
{
    uint32_t *pc = (uint32_t *)user_data;
	uint32_t stack_used;
    *pc += 1;
    /* discard const qualifier */
    struct k_thread *tid = (struct k_thread *)thread;
    
	stack_used = calc_used_stack((uint8_t *)thread->stack_info.start, thread->stack_info.size);

    printk("%02u id: (0x%08x) priority: %3d name: '%s' (%u/%u)", *pc, (uint32_t)tid,
	       k_thread_priority_get(tid), log_strdup(k_thread_name_get(tid)), stack_used, thread->stack_info.size);
#if 0 /* not in this zephyr version. */
    printk("state %s ", k_thread_state_str(tid));
#endif
    printk("\r\n");
}

static int cmd_sys_settime(const struct shell *shell, size_t argc, char **argv)
{
    char *p;
    cliSetTime_work.time = 0;
    cliSetTime_work.resynchTestCnt = 0;
    cliSetTime_work.resynchTestTimerPeriod = 0;

    if (argc > 1)
    {
        cliSetTime_work.time = (uint32_t)strtoul(argv[1], &p, 0);
    }
    
    if (argc > 2)
    {
        cliSetTime_work.resynchTestCnt = (uint32_t)strtol(argv[2], &p, 0);
    }

    if (argc > 3)
    {
        cliSetTime_work.resynchTestTimerPeriod = (uint32_t)strtoul(argv[3], &p, 0);
    }

    k_work_submit(&cliSetTime_work.work);

    return 0;
}

static int cmd_sys_gettime(const struct shell *shell, size_t argc, char **argv)
{
    SysTime_t currTime;

    currTime = SysTimeGet();
    LOG_INF("currTime: %d.%03d (0x%x)", currTime.Seconds, currTime.SubSeconds, currTime.Seconds);

    return 0;
}

static int cmd_sys_prtThreads(const struct shell *shell, size_t argc, char **argv)
{
    print_thread_list();
    return 0;
}

static int cmd_sys_accel_ctrl(const struct shell *shell, size_t argc, char **argv)
{
    char *p;

    if (argc > 1)
    {
        int ctrl = (uint32_t)strtoul(argv[1], &p, 0);
        if(ctrl == 0)
        {
            stopAccelSampling();
        }
        else
        {
            startAccelSampling();
        }
    }
    
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sys,
                               SHELL_CMD(settime, NULL, "Set time", cmd_sys_settime),
                               SHELL_CMD(gettime, NULL, "Get time", cmd_sys_gettime),
                               SHELL_CMD(threads, NULL, "Print Thread info", cmd_sys_prtThreads),
                               SHELL_CMD(acc, NULL, "Start/stop accel sampling", cmd_sys_accel_ctrl),
                               SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(sys, &sub_sys, "System commands", NULL);

