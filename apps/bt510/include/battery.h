/******************************************************************************
 * @file battery.h
 * @brief battery measurement
 * 
 * see .c file for details
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/

#ifndef __BATTERY_H__
#define __BATTERY_H__

#define LOW_BATT_THRESHOLD_MV_MINIMUM   (2050)
#define LOW_BATT_THRESHOLD_MV_MAX       (2750)
#define LOW_BATT_THRESHOLD_MV_DISABLE   (0)
#define LOW_BATT_THRESHOLD_MV_DEF       (2100)
#define BATT_LOW_LATCH_SAMPLE_COUNT     (5)

uint32_t ADC_SampleBatteryMv(void);
uint32_t ADC_GetLastBatteryMv(void);
bool ADC_IsBatteryLow();

#endif
