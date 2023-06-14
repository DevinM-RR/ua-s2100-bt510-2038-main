/******************************************************************************
 * @file temperature.c
 * 
 * @brief functions related to temperature measurement
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/
#include <zephyr.h>
#include <inttypes.h>
#include <device.h>
#include <drivers/sensor.h>

/* register this module for logging */
#define LOG_LEVEL LOG_LEVEL_DBG
#define LOG_MODULE_NAME temperature
#include <logging/log.h>
LOG_MODULE_REGISTER(temperature);

#include "temperature.h"

static const struct device *sensor;
struct sensor_value temp;
double tempC = 0;

uint32_t TMP_Init(void)
{
    sensor = device_get_binding(DT_LABEL(DT_INST(0, silabs_si7055)));
    if (sensor == NULL)
    {
        printk("Temperature device not found.\n");
        return -EIO;
    }

    return 0;
}

int32_t TMP_ReadAmbient(void)
{
    int32_t status = 0;

    if (sensor != NULL)
    {
        status = sensor_sample_fetch(sensor);

        if (status)
        {
            printk("Temperature sample fetch error\n");
        }
        else
        {
            sensor_channel_get(sensor, SENSOR_CHAN_AMBIENT_TEMP, &temp);

#ifdef CONFIG_NEWLIB_LIBC_FLOAT_PRINTF
            LOG_DBG("Temperature: %.2f C\n", sensor_value_to_double(&temp));
#else
            LOG_DBG("Temperature (int): %d\n", temp.val1);
#endif
        }
    }
    else
    {
        status = -EIO;
    }

    return status;
}

struct sensor_value TMP_GetLatestAmbient(void)
{
    return temp;
}