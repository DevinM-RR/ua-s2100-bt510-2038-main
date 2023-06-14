/******************************************************************************
 * @file accel.c
 * 
 * @brief accelerometer interface functions
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/
#include "accel.h"
#include <stdio.h>
#include "systime.h"
#include "battery.h"
#include "smp.h"
#include "DataLog.h"
#include "Settings.h"
#include "led.h"
#include "temperature.h"
#include <random/rand32.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#define LOG_MODULE_NAME accel
#include <logging/log.h>
#include <logging/log_ctrl.h>
LOG_MODULE_REGISTER(accel);

/* RX thread structures */
#define ACCEL_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(accel_stack, ACCEL_STACK_SIZE);
struct k_thread accel_thread;
#define ACCEL_THREAD_PRIORITY K_PRIO_COOP(7)

#define INIT_DELAY_SEC_MAX 1800 //30 minutes
// #define PRINT_ACCEL_SAMPLES

#define CHECK_BATTERY_DELAY_MS 10

static void accel_run(void);
static int read_accelerometer(const struct device *dev);

static void checkBattery(struct k_work *work);
static struct k_delayed_work checkBattery_work;

sample_t accelData[MAX_SAMPLE_CNT];
static int sampleCnt = 0;
static int fifoOverrunCnt = 0;
static bool samplingEnabled = true;

//raw * (1/64 * 16 / 1000 * 9.8) = 0.0098f //accel sensitivity from datasheet 16mg/digit converted to Gs, multiplied by 9.8m/s/s
const double multiplier = 0.00245f; //10-bit, +/- 8G full scale

static void accel_run(void)
{
    const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dh)));

    if (dev == NULL)
    {
        LOG_ERR("accel_run Could not get %s device",
                DT_LABEL(DT_INST(0, st_lis2dh)));
    }

    //Use a random delay on startup to allow bt to start first and 
    //space out sampling between multiple devices so they don't always have data at the same time
    //and allow some time for clock to be set by MG100
    uint32_t initialDelaySec = (sys_rand32_get() % (INIT_DELAY_SEC_MAX)) + 20;
    LOG_INF("Initial delay %d seconds", initialDelaySec);
    power_down_accelerometer();//disable accelerometer between sampling intervals
    k_sleep(K_SECONDS(initialDelaySec));

    while (true)
    {
        if (!smp_dfu_in_progress() && smp_bluetooth_is_started())
        {
            if (samplingEnabled)
            {
                power_up_accelerometer();
                k_sleep(K_SECONDS(1));

                //If accel is null, try to get it
                if (dev == NULL)
                {
                    dev = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dh)));

                    if (dev == NULL)
                    {
                        LOG_ERR("Could not get %s device",
                                DT_LABEL(DT_INST(0, st_lis2dh)));
                        k_sleep(K_MSEC(10000));
                        continue;
                    }
                }

                //Start next collection (only after previous dataset was stored)
                if (DataLogStartDataCollectionCb())
                {
                    //Reset counts for next run
                    fifoOverrunCnt = 0;
                    sampleCnt = 0;
                    memset(accelData, 0, sizeof(accelData));

                    printk("Start reading\n");
                    SysTime_t currTime = SysTimeGet();
                    int numSamples = read_accelerometer(dev);
                    // SysTime_t currTime2 = SysTimeGet();
                    printk("Stop reading\n");

                    SysTime_t currTime3 = SysTimeGet();
                    if (numSamples > 0)
                    {
                        printf("Sampling started at %d.%d\n", currTime.Seconds, currTime.SubSeconds);

#ifdef PRINT_ACCEL_SAMPLES
                        printf("Sample Num, X-raw, Y-raw, Z-raw, X, Y, Z\n");
                        for (int i = 0; i < MAX_SAMPLE_CNT; i++)
                        {
                            double xms2 = accelData[i].x * multiplier;
                            double yms2 = accelData[i].y * multiplier;
                            double zms2 = accelData[i].z * multiplier;
                            printf("%u,%d,%d,%d,%10.6f,%10.6f,%10.6f\n", i, accelData[i].x, accelData[i].y, accelData[i].z, xms2, yms2, zms2);
                        }
#endif
                        currTime3 = SysTimeGet();
                    }

                    if (is_set_ble_ad_flag_bit(ADV_FLAGS_HAS_EPOCH_TIME))
                    {
                        DataLogAddDataSet((uint8_t *)&accelData, currTime.Seconds);
                    }
                    else
                    {
                        //Call this function just to give back the semaphore
                        DataLogDataCollectCompleteCb();
                    }

                    //Print out details about the sampling.
                    // printk("fifoOverrunCnt: %d\n", fifoOverrunCnt);
                    // printk("Read Time: %d\n",
                    //        (currTime2.Seconds * 1000 + currTime2.SubSeconds) - (currTime.Seconds * 1000 + currTime.SubSeconds));
                    // printk("Print Time: %d\n",
                    //        (currTime3.Seconds * 1000 + currTime3.SubSeconds) - (currTime2.Seconds * 1000 + currTime2.SubSeconds));

                }
            }
            else
            {
                printf("Accel sampling is currently disabled...\n");
            }
        }

        led_trigger_pattern(LED_PATTERN_ON_OK);
        k_delayed_work_submit(&checkBattery_work, K_MSEC(CHECK_BATTERY_DELAY_MS)); // check battery when LED is on (under load)
        
        int64_t accelSleepStart = k_uptime_get();
        uint32_t accelSamplingInterval = DEFAULT_ACCEL_SAMPLING_INTERVAL;
        SettingGetValue(ID_ACCEL_SAMPLING_INTERVAL, (uint8_t *)&accelSamplingInterval, sizeof(accelSamplingInterval));
        LOG_INF("Restarting sampling in %d seconds\n", accelSamplingInterval);

        power_down_accelerometer();//disable accelerometer between sampling intervals
        while(1)
        {
            k_sleep(K_SECONDS(30));

            //Read interval setting in case it was modified
            SettingGetValue(ID_ACCEL_SAMPLING_INTERVAL, (uint8_t *)&accelSamplingInterval, sizeof(accelSamplingInterval));
            int64_t elapsed = (k_uptime_get() - accelSleepStart) / 1000;//convert to seconds
            if (elapsed >= accelSamplingInterval)
            {
                // LOG_INF("%d: %d elapsed, restart", (uint32_t)accelSleepStart, (uint32_t)elapsed);
                break;
            }
            else
            {
                // LOG_INF("%d: %d elapsed, still sleep", (uint32_t)accelSleepStart, (uint32_t)elapsed);
            }
        }
    }
}

static int read_accelerometer(const struct device *dev)
{
    struct lis2dh_data *lis2dh;
    const struct lis2dh_config *cfg;
    int status;

    if (dev == NULL)
    {
        LOG_ERR("accel dev not found: %s",
                DT_LABEL(DT_INST(0, st_lis2dh)));
        return -ENXIO;
    }

    lis2dh = dev->data;
    cfg = dev->config;

    lis2dh->bus = device_get_binding(cfg->bus_name);
    if (!lis2dh->bus)
    {
        LOG_ERR("master not found: %s", cfg->bus_name);
        return -ENXIO;
    }

    //keep reading until accelData buffer is filled
    while (sampleCnt < MAX_SAMPLE_CNT)
    {
        uint8_t fifoSrc;
        status = lis2dh->hw_tf->read_data(dev, LIS2DH_REG_FIFO_SRC,
                                          &fifoSrc,
                                          sizeof(fifoSrc));

        if (status < 0)
        {
            LOG_WRN("Could not read LIS2DH_REG_FIFO_SRC");
            return status;
        }

        //Read out the number of unread samples in fifo
        int numSamples = (fifoSrc & LIS2DH_FIFO_SRC_FSS_MASK);
        if (numSamples == 0)
        {
            continue;
        }

        if (fifoSrc & LIS2DH_FIFO_SRC_OVR)
        {
            fifoOverrunCnt++;
        }
        if (sampleCnt + numSamples >= MAX_SAMPLE_CNT)
        {
            numSamples = MAX_SAMPLE_CNT - sampleCnt;
        }

        status = lis2dh->hw_tf->read_data(dev, LIS2DH_REG_ACCEL_X_LSB,
                                          (uint8_t *)&accelData[sampleCnt],
                                          numSamples * sizeof(sample_t));

        if (status < 0)
        {
            LOG_WRN("Could not read accel axis data");
            return status;
        }
        sampleCnt += numSamples;
    }

    return sampleCnt;
}

int accel_init(void)
{
        k_delayed_work_init(&checkBattery_work,
                            checkBattery);
                            
    int status = power_up_accelerometer();
    if (!status)
    {
        LOG_DBG("Accel initialized");

        k_thread_name_set(
            k_thread_create(&accel_thread, accel_stack,
                            K_THREAD_STACK_SIZEOF(accel_stack),
                            (k_thread_entry_t)accel_run, NULL, NULL, NULL,
                            ACCEL_THREAD_PRIORITY, 0, K_NO_WAIT),
            "accel");
    }
    else
    {
        LOG_ERR("power_up_accelerometer failed. thread not started!");
    }

    return status;
}

int power_down_accelerometer(void)
{
#if defined(CONFIG_BOARD_BT510)
    const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dh)));

    if (dev == NULL)
    {
        LOG_ERR("power_down_accelerometer accel device is null!");
        return -EINVAL;
    }

    struct lis2dh_data *lis2dh = dev->data;
    const struct lis2dh_config *cfg = dev->config;
    int status;
    uint8_t id;

    lis2dh->bus = device_get_binding(cfg->bus_name);
    if (!lis2dh->bus)
    {
        LOG_ERR("master not found: %s", cfg->bus_name);
        return -EINVAL;
    }

    cfg->bus_init(dev);

    status = lis2dh->hw_tf->read_reg(dev, LIS2DH_REG_WAI, &id);
    if (status < 0)
    {
        LOG_ERR("Failed to read chip id.");
        return status;
    }

    if (id != LIS2DH_CHIP_ID)
    {
        LOG_ERR("Invalid chip ID: %02x", id);
        return -EINVAL;
    }

    status = lis2dh->hw_tf->update_reg(dev, LIS2DH_REG_CTRL0,
                                       LIS2DH_SDO_PU_DISC_MASK,
                                       LIS2DH_SDO_PU_DISC_MASK);
    if (status < 0)
    {
        LOG_ERR("Failed to disconnect SDO/SA0 pull-up.");
        return status;
    }
    else
    {
        printk("good disconnect SDO/SA0 pull-up.");
    }

    /* set accel to powerdown mode */
    uint8_t odr = 0x00;
    uint8_t accel_enable_bits = 0x00;
    uint8_t low_power_bit = 0x08;
    return lis2dh->hw_tf->write_reg(dev, LIS2DH_REG_CTRL1,
                                    odr | low_power_bit | accel_enable_bits);
#else
    return 0;
#endif
}

int power_up_accelerometer(void)
{
#if defined(CONFIG_BOARD_BT510)
    const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dh)));

    if (dev == NULL)
    {
        LOG_ERR("power_up_accelerometer accel device is null!");
        return -EINVAL;
    }

    struct lis2dh_data *lis2dh = dev->data;
    const struct lis2dh_config *cfg = dev->config;
    int status;
    uint8_t id;

    lis2dh->bus = device_get_binding(cfg->bus_name);
    if (!lis2dh->bus)
    {
        LOG_ERR("master not found: %s", cfg->bus_name);
        return -EINVAL;
    }

    cfg->bus_init(dev);

    status = lis2dh->hw_tf->read_reg(dev, LIS2DH_REG_WAI, &id);
    if (status < 0)
    {
        LOG_ERR("Failed to read chip id.");
        return status;
    }

    if (id != LIS2DH_CHIP_ID)
    {
        LOG_ERR("Invalid chip ID: %02x", id);
        return -EINVAL;
    }

    status = lis2dh->hw_tf->update_reg(dev, LIS2DH_REG_CTRL0,
                                       LIS2DH_SDO_PU_DISC_MASK,
                                       LIS2DH_SDO_PU_DISC_MASK);
    if (status < 0)
    {
        LOG_ERR("Failed to disconnect SDO/SA0 pull-up.");
        return status;
    }

    /* set accel to normal mode 1.344 kHz */
    uint8_t odr = 0x90; //1.344 kHz (normal mode)
    uint8_t accel_enable_bits = 0x07;
    uint8_t low_power_bit = 0x00; //normal mode (10-bit)
    status = lis2dh->hw_tf->write_reg(dev, LIS2DH_REG_CTRL1,
                                      odr | low_power_bit | accel_enable_bits);
    if (status < 0)
    {
        LOG_ERR("Failed to set ODR");
        return status;
    }

    /* Enable the high-pass filter to remove gravity component */
    status = lis2dh->hw_tf->write_reg(dev, LIS2DH_REG_CTRL2,
                                      0x09);
    if (status < 0)
    {
        LOG_ERR("Failed to set high pass filter");
        return status;
    }

    /* set accel full scale to +/- 8G */
    uint8_t bdu = 0x00;
    uint8_t ble = 0x00;
    uint8_t fsmode = 0x20; //+/- 8G
    uint8_t hr = 0x00;     //normal mode (10-bit)
    uint8_t st = 0x00;
    uint8_t sim = 0;
    status = lis2dh->hw_tf->write_reg(dev, LIS2DH_REG_CTRL4,
                                      bdu | ble | fsmode | hr | st | sim);
    if (status < 0)
    {
        LOG_ERR("Failed to set full scale");
        return status;
    }

    /* set FIFO enabled */
    uint8_t fifo = 0x40;
    status = lis2dh->hw_tf->write_reg(dev, LIS2DH_REG_CTRL5,
                                      fifo);
    if (status < 0)
    {
        LOG_ERR("Failed to set fifo enable");
        return status;
    }

    /* set FIFO mode - stream mode*/
    uint8_t fm = 0x80;
    // no interrupt configured for now
    status = lis2dh->hw_tf->write_reg(dev, LIS2DH_REG_FIFO_CTRL,
                                      fm);
    if (status < 0)
    {
        LOG_ERR("Failed to set fifo mode");
        return status;
    }

    return 0;
#else
    return 0;
#endif
}

void startAccelSampling()
{
    samplingEnabled = true;
    printk("Accel sampling enabled\n");
}

void stopAccelSampling()
{
    samplingEnabled = false;
    printk("Accel sampling disabled\n");
}

static void checkBattery(struct k_work *work)
{
    //Sampling Battery voltage should only be done under load.
    //This work item should be called right after turning on
    //the heartbeat LED
    uint32_t battmv = ADC_SampleBatteryMv();
    smp_set_battmv(battmv);
    LOG_DBG("batt mv: %d\n", battmv);

    TMP_ReadAmbient();
    struct sensor_value temp = TMP_GetLatestAmbient();
    smp_set_temp(temp.val1);
}
