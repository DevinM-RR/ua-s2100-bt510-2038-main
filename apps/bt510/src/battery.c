/******************************************************************************
 * @file battery.c
 * 
 * @brief functions related to battery measurement
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/
#include <zephyr.h>
#include <drivers/gpio.h>
#include <inttypes.h>
#include <device.h>
#include <nrfx_saadc.h>

/* register this module for logging */
#define LOG_LEVEL LOG_LEVEL_DBG
#define LOG_MODULE_NAME battery
#include <logging/log.h>
LOG_MODULE_REGISTER(battery);

#include "battery.h"

#define SAADC_PRIORITY                (5)
#define ADC_BATT_CHANNEL              (0)

#define FILTER_TIME_CONSTANT          (1)  // Time constant for time based math filter
#define SAMPLE_COUNTS                 (8)  // Number of samples taken each burst
#define MAX_ADC_READINGS              (1)

#define VOLTS_TO_CODE                 (284)     // Number of ADC per volt
#define BATTERY_ADVERT_OFFSET         (512)     // 1.8v offset value

#define SHUTDOWN_VOLTAGE_VOLTS        (2)
#define NEW_BATTERY_VOLTAGE_VOLTS     (3)
#define BATTERY_SEED                  (199)     // 3.2v seed value
#define SHUTDOWN_VOLTAGE_CODES        (SHUTDOWN_VOLTAGE_VOLTS * VOLTS_TO_CODE)
#define NEW_BATT_VOLTAGE_CODES        (NEW_BATTERY_VOLTAGE_VOLTS  * VOLTS_TO_CODE)
#define NEW_BATT_VOLT_MATH            (NEW_BATT_VOLTAGE_CODES * FILTER_TIME_CONSTANT)
#define MILLIVOLT_MULTIPLIER          (1000)

#define ADC_SIMPLE_MODE_CHANNEL_MASK 0x00000001
#define ADC_SIMPLE_MODE_RESOLUTION   NRF_SAADC_RESOLUTION_10BIT
#define ADC_SIMPLE_MODE_OVERSAMPLING NRF_SAADC_OVERSAMPLE_DISABLED

typedef struct 
{
  uint32_t batteryReading;
  uint32_t batteryVoltage;
  uint32_t batteryFilter;
  uint32_t batteryMath;
 } AdcControl_t;

 AdcControl_t adcControl;

static uint32_t TakeAdcReading(void);
static uint32_t batteryVoltageCalc(uint32_t adcTotal);

// setup single ended operation for ADC_BATT_CHANNEL to measure VDD
nrfx_saadc_channel_t batt_adc_channel =
    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_VDD, ADC_BATT_CHANNEL);

static nrf_saadc_value_t adc_buf[MAX_ADC_READINGS];
static uint32_t last_sampled_millivolts = 0;
static uint16_t consecutive_battery_low_readings = 0;
static bool latched_low_battery_state = false;

uint32_t ADC_SampleBatteryMv(void)
{
    last_sampled_millivolts = TakeAdcReading();
    //Reset battery math
    adcControl.batteryMath = 0;
    return(last_sampled_millivolts);
}

uint32_t ADC_GetLastBatteryMv(void)
{
    return(last_sampled_millivolts);
}

/*
 * read the ADC to determine battery voltage and map the voltage
 * to a normalized percentage between 0 - 100.
 */
static uint32_t TakeAdcReading(void)
{
	int ret, i;

    adcControl.batteryVoltage = NEW_BATT_VOLTAGE_CODES;
    adcControl.batteryFilter = NEW_BATT_VOLT_MATH;
    adcControl.batteryMath = 0;

    // enable SAADC
    ret = nrfx_saadc_init(SAADC_PRIORITY);
    if(ret != NRFX_SUCCESS) {
        LOG_ERR("Error setting up battery measurement");
        return 0xFF;
    }

    // setup ADC channel 0 for reading VDD
    ret = nrfx_saadc_channels_config(&batt_adc_channel, 1);
    if(ret != NRFX_SUCCESS) {
        LOG_ERR("Error configuring battery channel");
        return 0xFF;
    }

    // setup simple mode with no callback (blocking reads with nrfx_saadc_mode_trigger calls)
    ret = nrfx_saadc_simple_mode_set(ADC_SIMPLE_MODE_CHANNEL_MASK,
                                    ADC_SIMPLE_MODE_RESOLUTION,
                                    ADC_SIMPLE_MODE_OVERSAMPLING,
                                    NULL);

    // setup the buffer to store ADC samples
    ret = nrfx_saadc_buffer_set(adc_buf, MAX_ADC_READINGS);

    // take several readings to be filtered
    for(i = 0; i < SAMPLE_COUNTS; i++)
    {
        // trigger the ADC reading
        nrfx_saadc_mode_trigger();

        adcControl.batteryMath += (adc_buf[0] & 0x03FF);
    }

    // disable SAADC
    nrfx_saadc_uninit();

    //printk("adc: %d\n", adc_buf[0]);

    return(batteryVoltageCalc(adcControl.batteryMath));

    return 0xFF;
}

static uint32_t batteryVoltageCalc(uint32_t adcTotal)
{
  uint32_t milliVolt;
  
  // Time filtered battery
  adcTotal /= SAMPLE_COUNTS;
  adcControl.batteryFilter -= adcControl.batteryVoltage;
  adcControl.batteryFilter += adcTotal;
  adcControl.batteryVoltage = adcControl.batteryFilter / FILTER_TIME_CONSTANT;
  milliVolt = adcControl.batteryVoltage * MILLIVOLT_MULTIPLIER;
  milliVolt = milliVolt / VOLTS_TO_CODE;
  
  //LOG_INF("Battery Voltage %d", milliVolt);
  return(milliVolt);
}

/*
 * return whether the battery is low based on the lowBattThresholdMv setting
 */
bool ADC_IsBatteryLow()
{
    uint16_t lowBattThresholdMv = LOW_BATT_THRESHOLD_MV_DEF;

    /* handle the boot-up case where no samples have been taken yet */
    if(last_sampled_millivolts == 0)
    {
        return false;
    }

    /* if the last sample is less than lowBattThresholdMv,
     * increment the counter otherwise clear to zero */
    if(last_sampled_millivolts < lowBattThresholdMv)
    {
        consecutive_battery_low_readings++;
    } else {
        consecutive_battery_low_readings = 0;
    }

    /* set the latched_low_battery_state if battery has been
     * below threshold for BATT_LOW_LATCH_SAMPLE_COUNT consecutive samples */
    if(consecutive_battery_low_readings >= BATT_LOW_LATCH_SAMPLE_COUNT)
    {
        latched_low_battery_state = true;
    }

    return latched_low_battery_state;
}