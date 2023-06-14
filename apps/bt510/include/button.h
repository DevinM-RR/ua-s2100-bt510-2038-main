/******************************************************************************
 * @file button.h
 * @brief button handling
 * 
 * see .c file for details
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/

#ifndef __BUTTON_H__
#define __BUTTON_H__
#include <nrfx_gpiote.h>
#include <drivers/gpio.h>

#if defined(CONFIG_BOARD_BL654_DVK)

#define BUTTON_1_PIN 11

#elif defined(CONFIG_BOARD_BT510)

#define BUTTON_1_PIN 42

#endif


void button_init(gpio_callback_handler_t button_pressed);
int button_read(void);
void button_init_gpiote(nrfx_gpiote_evt_handler_t button_pressed);
void button_set_next_edge(int sense, nrfx_gpiote_evt_handler_t button_pressed);

#endif