//=================================================================================================
//! @file DataLog.h
//!
//! @brief This is the header file for DataLog.c.  It contains global
//!  function prototypes and application for the Data Log definitions.
//!
//! @copyright � 2020 Laird Connectivity, Inc
//!            Not for further distribution. All rights reserved.
//=================================================================================================

#ifndef DATA_LOG_H
#define DATA_LOG_H

#include "smp.h"
#include "systime.h"
#include "accel.h"
#include <fs/fcb.h>

#ifdef __cplusplus
extern "C" {
#endif

//=================================================================================================
// Includes
//=================================================================================================


//=================================================================================================
// Global Constants, Macros and Type Definitions
//=================================================================================================

#define DLOG_RECORD_TYPE_DEFAULT                         DLOG_RECORD_TYPE_ACCEL
#define DLOG_RECORD_TYPE_AD_RSSI_TRACKING                0x10
#define DLOG_RECORD_TYPE_AD_RSSI_TRACKING_W_TS           0x11
#define DLOG_RECORD_TYPE_ACCEL                           0x20

#define DLOG_LOG_INTERVAL_MIN_DEF       0
#define DLOG_CONTACT_PERIOD_SEC_DEF     86400 //24 hours
#define DLOG_DISCOVERY_DURATION_SEC_DEF 300 //5 minutes
#define DLOG_TRACKING_DURATION_SEC_DEF  1800 //30 minutes
#define DLOG_MAINTENANCE_PERIOD_SEC_DEF 300 //5 minutes
#define DLOG_OVERWRITE_FULL_DATALOG_DEF 1
#define DLOG_FULL_LOG_THRESHOLD_DEF     336 //entry count when there is 1 fully erased page in datalog
#define LOG_ENTRY_SIZE                  256


#if (DLOG_RECORD_TYPE_DEFAULT == DLOG_RECORD_TYPE_AD_RSSI_TRACKING)
    #define RECORD_SIZE sizeof(log_entry_data_rssi_tracking_t)
#elif (DLOG_RECORD_TYPE_DEFAULT == DLOG_RECORD_TYPE_AD_RSSI_TRACKING_W_TS)
    #define RECORD_SIZE sizeof(log_entry_data_rssi_tracking_w_ts_t)
#elif (DLOG_RECORD_TYPE_DEFAULT == DLOG_RECORD_TYPE_ACCEL)
    #define RECORD_SIZE sizeof(log_entry_data_accel_t)
#else
#warning "Unknown default record type selected"
#endif
#define LOG_ENTRY_MAX_NUM_RECORDS           ((LOG_ENTRY_SIZE - sizeof(log_entry_header_t)) / (RECORD_SIZE))

#define RSSI_THRESHOLD_DEF 0 //unused but needed for BLE transfer protocol
#define HAS_DATA_THRESHOLD_DEF          1

#define LOG_ADDR_START FLASH_AREA_OFFSET(log)

#define LOG_ENTRY_IDX_START_BYTE 0
#define LOG_ENTRY_IDX_FLAGS 1
#define LOG_ENTRY_IDX_INTERVAL 2
#define LOG_ENTRY_IDX_DEVICE_ID 3
#define LOG_ENTRY_IDX_TIMESTAMP 4

#define LOG_ENTRY_START_BYTE 0xA5
#define LOG_ENTRY_FLAGS_UNSENT 0xFF
#define LOG_ENTRY_FLAGS_SENT 0x00

#define LOG_ENTRY_PROTOCOL_VERSION 0x0003

/* data entry data */
typedef struct __attribute__((packed)) __log_entry_data_rssi_tracking_t__
{
    uint8_t recordType;
    int8_t rssi;
    uint8_t motion;
    int8_t txPower;

} log_entry_data_rssi_tracking_t;

/* data entry data */
typedef struct __attribute__((packed)) __log_entry_data_rssi_tracking_w_ts_t__
{
    uint8_t recordType;
    uint8_t status;
    uint8_t reserved1;
    uint16_t scanIntervalOffset;
    int8_t rssi;
    uint8_t motion;
    int8_t txPower;

} log_entry_data_rssi_tracking_w_ts_t;

/* data entry data */
typedef struct __attribute__((packed)) __log_entry_data_accel_t__
{
    uint8_t recordType;
    uint8_t reserved;
    sample_t xyz;

} log_entry_data_accel_t;

typedef struct __attribute__((packed)) __LOG_ENTRY_HEADER_T__
{
    uint8_t entryStart;
    uint8_t flags;
    uint16_t sampleCount;
    uint8_t serial[BT_MAC_ADDR_LEN];
    uint32_t timestamp;
    uint16_t entrySize;//not written to flash, but set during data transfer to gateway
} log_entry_header_t;

/* Log entry item */
typedef struct __attribute__((packed)) __LOG_ENTRY_T__
{
    log_entry_header_t header;

    union __attribute__((packed))
    {        
        uint8_t recordType; //all data structures need to include type as first byte
        log_entry_data_rssi_tracking_t rssiTrackData[LOG_ENTRY_MAX_NUM_RECORDS];
        log_entry_data_rssi_tracking_w_ts_t rssiTrackWTsData[LOG_ENTRY_MAX_NUM_RECORDS];
        log_entry_data_accel_t accelData[LOG_ENTRY_MAX_NUM_RECORDS];
    } data;

} log_entry_t;

/* data stored to a log entry by this device itself to record some of its status values.
 * This roughly equates to values that would end up in a "thing shadow" for this device.
 */
struct ct_record_local {
    uint8_t fw_version[4];             // 0
    uint16_t network_id;               // 4
    uint16_t ad_interval_ms;           // 6
    int32_t temperature_int;            // 8
    int32_t temperature_frac;           // 12
    uint8_t battery_level;             // 16
    uint8_t deviceType;                // 17
    int8_t tx_power;                  // 18
    uint32_t up_time_sec;              // 19
} __attribute__((packed));

struct ct_log_header {
    uint16_t entry_protocol_version;   // 2
    uint16_t entry_size;               // 4
    uint16_t entry_count;              // 6
    uint8_t device_id[6];              // 12
    uint32_t device_time;              // 16
    uint32_t log_size;                 // 20
    uint32_t last_upload_time;         // 24
    struct ct_record_local local_info;
} __attribute__((packed));

//=================================================================================================
// Global Data Definitions
//=================================================================================================

//=================================================================================================
// Global Function Prototypes
//=================================================================================================
/**
 * @brief  Init the Data Log module.
 *
 */
void DataLogInit(void);

/**
 * @brief  Perform required actions in datalog before starting scan.
 *
 */
bool DataLogStartDataCollectionCb(void);

/**
 * @brief  Add dataset to datalog
 * 
 * @param pData - pointer to data to be stored
 * @param timestampSec - timestamp of when accel sampling started
 */
void DataLogAddDataSet(uint8_t *pData, uint32_t timestampSec);

/**
 * @brief  Initiate start of datalog download. Internally, creates snapshot of current datalog info used for datalog download.
 * Takes the semamphore to prevent adding new entries.
 * 
 * @return Return size of log (unacked entries)
 *  
 */
uint32_t DataLogStartDownload(void);

/**
 * @brief  Reset datalog download. Internally, resets snapshot of current datalog info used for datalog download.
 * 
 */
void DataLogResetDownload(void);

/**
 * @brief  Get next unacked data log entry
 *
 * @param pEntryAddr - returned flash address of next unacked entry
 * @param pDataOut - pointer to buffer that will be filled with entry data (pointed to by pEntryAddr)
 * @param pEntrySize - returned flash size of next unacked entry
 * @param available_len - space available in buffer
 * 
 * @return Return 0 on success, -1 on failure
 *  
 */
uint32_t DataLogGetNextEntry(struct fcb_entry *pPrevEntryLoc, uint8_t *pDataOut, uint16_t *pEntrySize, uint32_t available_len);

/**
 * @brief  Ack all entries that were retrieved in from DataLogGetNextEntry
 * 
 * @param forceAll - set to true to ack all entries in log
 */
void DataLogAckEntries(uint32_t forceAll);

/**
 * @brief  Get total size of unacked log entries
 *
 * @return size - (num unacked entries * LOG_ENTRY_SIZE)
 */
uint32_t DataLogGetLogSize(void);

/**
 * @brief  Get number of unacked log entries from datalog
 *
 * @return num unacked entries
 */
uint32_t DataLogGetUnackedEntryCount(void);

/**
 * @brief  Get number of unacked log entries from download snapshot info
 *
 * @return num unacked entries
 */
uint32_t DataLogGetDownloadEntryCount(void);

/**
 * @brief  Completes log read procedure (ack's all pending events)
 *
 */
void DataLogReadComplete(void);

/**
 * @brief  Aborts the log read procedure (discards any pending Ack events)
 */
void DataLogReadAbort(void);

/**
 * @brief  Function that should be called after datalog was read.
 * Starts a timer that prevents HAS DATA flag from being set.
 */
void DataLogWasRead(void);

/**
 * @brief  Erase the oldest entry of the data log
 */
void DataLogErase(void);

/**
 * @brief  Erase the entire data log
 */
void DataLogEraseAll(void);

/**
 * @brief  Stop the log interval timer and immediately update has data flag in ad
 */
void DataLogStopLogIntervalTimer(void);

/**
 * @brief  Updates the has log data flag in the advertisement based on state of the datalog 
 */
void UpdateHasDataFlag(void);

/**
 * @brief  Performs any required datalog activities when time is set
 */
void DataLogTimeSetCb(uint32_t oldTime, uint32_t newTime);

/**
 * @brief  Performs any required datalog activities on disconnection
 * 
 */
void DataLogDisconnectDb(void);

/**
 * @brief Add test entry to the datalog with 0xFFFFFFFFFFFF test address
 * 
 * @param numRecords - number of records to add
 */
void DataLogAddTestEntry(uint32_t numRecords);

/**
 * @brief Get the datalog Log interval
 * 
 * @return log interval
 */
uint16_t get_log_interval(void);

/**
 * @brief Call this function when data collection is completed but
 * dataset cannot be added to log (mainly due to RTC not having been set)
 * This function's only use is to give back a semaphore.
 * 
 */
void DataLogDataCollectCompleteCb(void);

#ifdef __cplusplus
}
#endif

#endif // #ifndef DATA_LOG_H


//=================================================================================================
// end
//=================================================================================================
