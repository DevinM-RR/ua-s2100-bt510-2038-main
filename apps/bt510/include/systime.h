/*!
 * \file      systime.h
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
#ifndef __SYS_TIME_H__
#define __SYS_TIME_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include <time.h>

#define EPOCH_TIME_JAN_1_2020 1577836800

/*!
 * \brief Structure holding the system time in seconds and milliseconds.
 */
typedef struct systime_s
{
    uint32_t Seconds;
    int16_t  SubSeconds;
}SysTime_t;

/*!
 * \brief Init the RTC module
 */
void RtcInit();

/*!
 * \brief Sets new system time
 *
 * \param  systime    New seconds/sub-seconds since UNIX epoch origin
 * \retval true if RTC is updated, false if RTC is not (due to invalid time input)
 */
bool SysTimeSet(SysTime_t systime);

/*!
 * \brief Gets current system time
 *
 * \retval systime    Current seconds/sub-seconds since UNIX epoch origin
 */
SysTime_t SysTimeGet( void );

/*!
 * \brief Gets current system time (only seconds value)
 *
 * \retval systime    Current seconds since UNIX epoch origin
 */
uint32_t SysTimeGetSeconds(void);

#ifdef __cplusplus
}
#endif

#endif // __SYS_TIME_H__
