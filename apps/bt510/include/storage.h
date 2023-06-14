/******************************************************************************
 * @file storage.h
 * @brief storage related functions
 *
 * see .c file for details
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/

#ifndef __STORAGE_H__
#define __STORAGE_H__

#include <storage/flash_map.h>

#define FS_MOUNT_STR "/lfs"
#define SETTINGS_FILENAME "/lfs/params.txt"
#define SETTINGS_FILENAME_LEN 9
#define TIME_SET_FILENAME "/sys/time"
#define TIME_SET_FILENAME_LEN 9
#define TIME_SET_TXT_FILENAME "/sys/time.txt"
#define TIME_SET_TXT_FILENAME_LEN 13
#define LED_PATTERN_FILENAME "/sys/led/pattern"
#define LED_PATTERN_FILENAME_LEN 16
#define LED_PATTERN_TXT_FILENAME "/sys/led/pattern.txt"
#define LED_PATTERN_TXT_FILENAME_LEN 20

void storage_init(void);

#endif