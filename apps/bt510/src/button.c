/******************************************************************************
 * @file button.c
 * 
 * @brief button support for the tracker pushbutton. Configures an interrupt
 * triggered on both rising/falling edges and invokes a passed in callback
 * on state changes.
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/
#include <zephyr.h>
#include <drivers/gpio.h>
#include <inttypes.h>
#include <nrfx_gpiote.h>

/* register this module for logging */
#define LOG_LEVEL LOG_LEVEL_DBG
#define LOG_MODULE_NAME button
#include <logging/log.h>
LOG_MODULE_REGISTER(button);

#include "button.h"

/*
 * Devicetree helper macro which gets the 'flags' cell from a 'gpios'
 * property, or returns 0 if the property has no 'flags' cell.
 */

#define FLAGS_OR_ZERO(node)						\
	COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags),		\
		    (DT_GPIO_FLAGS(node, gpios)),			\
		    (0))

/*
 * Get button configuration from the devicetree sw0 alias.
 *
 * At least a GPIO device and pin number must be provided. The 'flags'
 * cell is optional.
 */

#define SW0_NODE	DT_ALIAS(sw0)

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
#define SW0_GPIO_LABEL	DT_GPIO_LABEL(SW0_NODE, gpios)
#define SW0_GPIO_PIN	DT_GPIO_PIN(SW0_NODE, gpios)
#define SW0_GPIO_FLAGS	(GPIO_INPUT | FLAGS_OR_ZERO(SW0_NODE))
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#define SW0_GPIO_LABEL	""
#define SW0_GPIO_PIN	0
#define SW0_GPIO_FLAGS	0
#endif


static struct gpio_callback button_cb_data;
static const struct device *button;

/*
 * init the button and register the passed in callback to
 * be invoked on button press and release during active mode
 */
void button_init(gpio_callback_handler_t button_pressed)
{
	int ret;

	button = device_get_binding(SW0_GPIO_LABEL);
	if (button == NULL) {
		LOG_ERR("Error: didn't find %s device\n", SW0_GPIO_LABEL);
		return;
	}

	/* explicitly disable gpiote if we are using gpio pin interrupt mode */
	if(nrfx_gpiote_is_init())
	{
		nrfx_gpiote_uninit();
	}

	ret = gpio_pin_configure(button, SW0_GPIO_PIN, SW0_GPIO_FLAGS);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d\n",
		       ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	ret = gpio_pin_interrupt_configure(button,
					   SW0_GPIO_PIN,
					   GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(SW0_GPIO_PIN));
	gpio_add_callback(button, &button_cb_data);
}

/*
 * read the state of the button's GPIO pin.
 * returns 0 if button is not pressed, 1 if pressed
 */
int button_read(void)
{
    int val;

    val = gpio_pin_get(button, SW0_GPIO_PIN);
    return !val; // invert so that 1 is returned when pressed
}

/*
 * set the sense state to trigger the next wake from "ship mode"
 */
void button_set_next_edge(int sense, nrfx_gpiote_evt_handler_t button_pressed)
{
	nrfx_err_t nrfxStatus;
	nrf_gpiote_polarity_t sense_state = NRF_GPIOTE_POLARITY_TOGGLE;

// Possible sense values:
// NRF_GPIOTE_POLARITY_LOTOHI
// NRF_GPIOTE_POLARITY_HITOLO
// NRF_GPIOTE_POLARITY_TOGGLE

	/* disable the GPIOTE PORT event so it can be reconfigured */
	nrfx_gpiote_in_event_disable(BUTTON_1_PIN);

	if(sense) {
		sense_state = NRF_GPIOTE_POLARITY_HITOLO;
	} else {
		sense_state = NRF_GPIOTE_POLARITY_LOTOHI;
	}

	/* Configure to generate PORT event (wakeup) on button 1 press/release. */
	nrfx_gpiote_in_config_t inputConfig = 
	{
		.sense = sense_state,
		.pull = NRF_GPIO_PIN_PULLUP,
		.is_watcher = false,
		.hi_accuracy = false,
		.skip_gpio_setup = false
	};
	nrfxStatus = nrfx_gpiote_in_init(BUTTON_1_PIN, &inputConfig, button_pressed);
	if(nrfxStatus != NRFX_SUCCESS)
	{
		LOG_ERR("err %d configuring button", nrfxStatus);
		return;
	}

	/* enable PORT event interrupt */
	nrfx_gpiote_in_event_enable(BUTTON_1_PIN, true);
}

/*
 * init the button for exiting "ship mode"
 */
void button_init_gpiote(nrfx_gpiote_evt_handler_t button_pressed)
{
	nrfx_err_t nrfxStatus;

	button = device_get_binding(SW0_GPIO_LABEL);
	if (button == NULL) {
		LOG_ERR("Error: didn't find %s device\n", SW0_GPIO_LABEL);
		return;
	}

	if( !nrfx_gpiote_is_init() )
	{
		nrfxStatus = nrfx_gpiote_init(3);
		if(nrfxStatus != NRFX_SUCCESS) {
			LOG_ERR("err %d initializing gpiote", nrfxStatus);
			return;
		}
	}

	button_set_next_edge(1, button_pressed);
}
