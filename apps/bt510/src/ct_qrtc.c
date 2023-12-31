/**
 * @file ct_qrtc.c
 * @brief
 *
 * Copyright (c) 2020 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/******************************************************************************/
/* Includes                                                                   */
/******************************************************************************/
#include <zephyr.h>
#include <stdlib.h>
#include "ct_qrtc.h"
#include <time.h>

/******************************************************************************/
/* Local Constant, Macro and Type Definitions                                 */
/******************************************************************************/
struct qrtc {
	bool epochWasSet;
	uint32_t offset;
};

/******************************************************************************/
/* Local Data Definitions                                                     */
/******************************************************************************/
static struct qrtc qrtc;
K_MUTEX_DEFINE(qrtcMutex);

/******************************************************************************/
/* Local Function Prototypes                                                  */
/******************************************************************************/
static void UpdateOffset(uint32_t Epoch);
static uint32_t GetUptime(uint32_t *pMilliseconds);
static uint32_t GetUptimeSeconds(void);
// static uint32_t ConvertTimeToEpoch(time_t Time);

/******************************************************************************/
/* Global Function Definitions                                                */
/******************************************************************************/
void Qrtc_Init(void)
{
	qrtc.offset = 0;
	qrtc.epochWasSet = false;
}

uint32_t Qrtc_SetEpoch(uint32_t Epoch)
{
	UpdateOffset(Epoch);
	return Qrtc_GetEpoch();
}

//mktime not supported in standard Zephyr C-library 
// uint32_t Qrtc_SetEpochFromTm(struct tm *pTm, int32_t OffsetSeconds)
// {
// 	time_t rawTime = mktime(pTm);
// 	uint32_t epoch = ConvertTimeToEpoch(rawTime);
// 	/* (local + offset) = UTC */
// 	if (abs(OffsetSeconds) < epoch) {
// 		epoch -= OffsetSeconds;
// 		UpdateOffset(epoch);
// 	}
// 	return Qrtc_GetEpoch();
// }

uint32_t Qrtc_GetEpochWithMs(uint32_t *pMilliseconds)
{
    uint32_t seconds = GetUptime(pMilliseconds);

    return (seconds + qrtc.offset);
}

uint32_t Qrtc_GetEpoch(void)
{
	return (GetUptimeSeconds() + qrtc.offset);
}

bool Qrtc_EpochWasSet(void)
{
	return qrtc.epochWasSet;
}

/******************************************************************************/
/* Local Function Definitions                                                 */
/******************************************************************************/
/**
 * @brief Generate and save an offset using the current uptime to
 * make a quasi-RTC because there isn't hardware support.
 */
static void UpdateOffset(uint32_t Epoch)
{
	k_mutex_lock(&qrtcMutex, K_FOREVER);
	uint32_t uptime = GetUptimeSeconds();
	if (Epoch >= uptime) {
		qrtc.offset = Epoch - uptime;
		qrtc.epochWasSet = true;
	}
	k_mutex_unlock(&qrtcMutex);
}

static uint32_t GetUptime(uint32_t *pMilliseconds)
{
	int64_t uptimeMs = k_uptime_get();
	if (uptimeMs < 0) {
		return 0;
	}

    *pMilliseconds = uptimeMs % MSEC_PER_SEC;

	return (uint32_t)(uptimeMs / MSEC_PER_SEC);
}

static uint32_t GetUptimeSeconds(void)
{
	int64_t uptimeMs = k_uptime_get();
	if (uptimeMs < 0) {
		return 0;
	}
	return (uint32_t)(uptimeMs / MSEC_PER_SEC);
}

//No longer used since mktime not supported in standard Zephyr C-library 
// static uint32_t ConvertTimeToEpoch(time_t Time)
// {
// 	/* Time is a long long int in Zephyr. */
// 	if (Time < 0) {
// 		return 0;
// 	} else if (Time >= UINT32_MAX) {
// 		return 0;
// 	} else {
// 		return (uint32_t)Time;
// 	}
// }
