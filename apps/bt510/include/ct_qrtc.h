/**
 * @file ct_qrtc.h
 * @brief Quasi Real Time Clock that uses offset and system ticks.
 *
 * Copyright (c) 2020 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __CT_QRTC_H__
#define __CT_QRTC_H__

/******************************************************************************/
/* Includes                                                                   */
/******************************************************************************/
#include <zephyr/types.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/* Global Function Prototypes                                                 */
/******************************************************************************/
/**
 * @brief Resets epoch to zero and sets epochWasSet to false.
 */
void Qrtc_Init(void);

/**
 * @brief Set the epoch
 *
 * @param Epoch in seconds from Jan 1, 1970
 *
 * @retval recomputed value for testing
 */
uint32_t Qrtc_SetEpoch(uint32_t Epoch);

/**
 * @brief Set the epoch using time structure.
 * (mktime not supported in standard Zephyr C-library)
 * 
 * @param pTm is a pointer to a time structure
 * @param OffsetSeconds is the offset in seconds from UTC that the time structure
 * contains.
 *
 * @note The cellular modem provides local time and an offset and it is much
 * easier to add the offset to the epoch than to adjust the time structure.
 *
 * @retval epoch for testing
 */
// uint32_t Qrtc_SetEpochFromTm(struct tm *pTm, int32_t OffsetSeconds);

/**
 * @brief Get current epoch time
 *
 * @param pMilliseconds - return current milliseconds
 * 
 * @retval Seconds since Jan 1, 1970.
 * 
 */
uint32_t Qrtc_GetEpochWithMs(uint32_t *pMilliseconds);

/**
 * @retval Seconds since Jan 1, 1970.
 */
uint32_t Qrtc_GetEpoch(void);

/**
 * @retval true if the epoch has been set, otherwise false
 */
bool Qrtc_EpochWasSet(void);

#ifdef __cplusplus
}
#endif

#endif /* __QRTC_H__ */
