/******************************************************************************
 * @file temperature.h
 * @brief temperature measurement
 * 
 * see .c file for details
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/

#ifndef __TEMPERATURE_H__
#define __TEMPERATURE_H__

uint32_t TMP_Init(void);
int32_t TMP_ReadAmbient(void);
struct sensor_value TMP_GetLatestAmbient(void);

#endif
