/*!
 * \file      systime.c
 *
 * \brief     System time functions implementation.
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2018 Semtech - STMicroelectronics
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    MCD Application Team ( STMicroelectronics International )
 */
#include <stdio.h>
#include <time.h>
#include <zephyr.h>
// #include "rtc-board.h"
#include "ct_qrtc.h"
#include "systime.h"
#include "smp.h"

#define LOG_LEVEL LOG_LEVEL_DBG
#define LOG_MODULE_NAME systime
#include <logging/log.h>
LOG_MODULE_REGISTER(systime);

void RtcInit()
{
    Qrtc_Init();
}

bool SysTimeSet( SysTime_t newTime )
{
    bool success = false;
    SysTime_t currTime = SysTimeGet();

    if(newTime.Seconds >= EPOCH_TIME_JAN_1_2020)
    {
        Qrtc_SetEpoch(newTime.Seconds);

        LOG_DBG("Time set from %d to %d", currTime.Seconds, newTime.Seconds);
        set_ble_ad_flag_bit(ADV_FLAGS_HAS_EPOCH_TIME);
        success = true;        
    }
    else
    {
        LOG_WRN("Invalid time received %d", newTime.Seconds);
        success = false;
    }

    return success;
}

SysTime_t SysTimeGet( void )
{
    SysTime_t systime = { .Seconds = 0, .SubSeconds = 0 };

    uint32_t ms;
    uint32_t seconds = Qrtc_GetEpochWithMs(&ms);

    systime.Seconds = seconds;
    systime.SubSeconds = ms;

    return systime;
}

uint32_t SysTimeGetSeconds( void )
{
    return SysTimeGet().Seconds;
}