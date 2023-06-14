/******************************************************************************
 * @file led.h
 * @brief LED pattern sequencing
 * 
 * see .c file for details
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/

#ifndef __LED_H__
#define __LED_H__

#define LED_NONE 0
#define LED_GREEN 0
#define LED_RED 1

#define LED_RGB_R       2
#define LED_RGB_G       3
#define LED_RGB_B       4
#define LED_RGB_WHITE   5
#define LED_RGB_YELLOW  6
#define LED_RGB_ORANGE  7
#define LED_RGB_CYAN    8
#define LED_RGB_GRAY    9

enum LED_PATTERN {
    LED_PATTERN_OFF,
    LED_PATTERN_ON_OK,
    LED_PATTERN_PRESS_2_SEC,
    LED_PATTERN_PROVISIONING_MODE,
    LED_PATTERN_PROVISION_COMPLETE,
    LED_PATTERN_CONNECTED,
    LED_PATTERN_DFU_COMPLETE,
    LED_PATTERN_POWER_ON,
    LED_PATTERN_LOW_BATT,
    LED_PATTERN_DFU_IN_PROGRESS,
    LED_PATTERN_FACTORY_RESET,
    LED_PATTERN_DATALOG_FULL,
    
    LED_PATTERN_COUNT /* must always be the last entry */
};

enum LED_OPERATION {
    LED_OP_END,
    LED_OP_OFF,
    LED_OP_ON,
    LED_OP_LOOP,
    LED_OP_RAMP_ON,
    LED_OP_RAMP_OFF
};

void led_init(void);
void led_trigger_pattern(enum LED_PATTERN pattern);
enum LED_PATTERN led_get_pattern(void);
bool led_sequence_is_running(void);

#endif