//=================================================================================================
//!
#define THIS_FILE "DataLog.c"
//!
//! @copyright � 2020 Laird Connectivity, Inc
//!            Not for further distribution. All rights reserved.
//=================================================================================================

/**
* \file DataLog.c
*
* \brief This file handles the data log implementation in internal Flash.
*/

//=================================================================================================
// Includes
//=================================================================================================
#include <zephyr.h>
#include <drivers/flash.h>
#include <device.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <logging/log.h>
#include <storage/flash_map.h>

#include <fs/fcb.h>

#include <stdlib.h>
#include <stdio.h>

#include "accel.h"
#include "DataLog.h"
#include "systime.h"
#include "smp.h"
#include "battery.h"

//=================================================================================================
// Local Constant, Macro and Type Definitions
//=================================================================================================
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(dlog);
/*
 * When DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL is available, we use it here.
 * Otherwise the device can be set at runtime with the set_device command.
 */
#ifndef DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL
#define DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL ""
#endif

#define FCB_FLASH_AREA_ID FLASH_AREA_ID(log)

#define LOG_SIZE FLASH_AREA_SIZE(log)
#define LOG_ADDR_END (LOG_ADDR_START + LOG_SIZE)

#define LOG_ENTRY_IDX_START_BYTE 0
#define LOG_ENTRY_IDX_FLAGS 1
#define LOG_ENTRY_IDX_INTERVAL 2
#define LOG_ENTRY_IDX_DEVICE_ID 3
#define LOG_ENTRY_IDX_TIMESTAMP 4

#define LOG_ENTRY_ERASED_BYTE 0xFF
#define LOG_ENTRY_START_BYTE 0xA5
#define LOG_ENTRY_FLAGS_UNACK 0x01        //Event unacked == BIT0 is set, Event acked == BIT0 is cleared
#define LOG_ENTRY_FLAGS_TIME_INVALID 0x02 //invalid time == BIT1 is set, valid time == BIT1 is cleared

#define LOG_ENTRY_HEADER_ACK_WRITE_SIZE 4

#define DB_NUM_DEVICES 352
#define DB_FLAG_IN_USE 0x0001
#define DB_FLAG_FOUND_IN_LATEST_SCAN 0x0002
#define DB_FLAG_RESET_FIRST_SEEN 0x0004
#define DB_STATE_DISCOVERY 1
#define DB_STATE_TRACKED 2
#define DB_STATE_UNTRACKED 3
#define DB_STATE_EXPIRED 4 //contact period expired. Needs to be set when recovering the deviceDB on startup
#define MAX_DISC_RECORDS 10

#define LOG_WRITE_SEM_TIMEOUT 1000

#define CLEAR_DB_NONE 0
#define CLEAR_DB_ENTRY_CONTACT_PERIOD 1
#define CLEAR_DB_ENTRY_LOG_ENTRY_MAX 2

#define DB_ENTRY_WRITE_NONE 0
#define DB_ENTRY_WRITE_NEW 1
#define DB_ENTRY_WRITE_UPDATE 2

#define FCB_SECTOR_SIZE 0x1000 /*4K*/

#define DATASET_NUM_ENTRIES ((MAX_SAMPLE_CNT + (LOG_ENTRY_MAX_NUM_RECORDS-1)) / LOG_ENTRY_MAX_NUM_RECORDS) //number of entries stored for single dataset (rounded up)

// #define TEST_ACCEL_DATA
sample_t testAccelData[MAX_SAMPLE_CNT];

//=================================================================================================
// Global Data Definitions
//=================================================================================================

//=================================================================================================
// Local Data Definitions
//=================================================================================================

static struct k_work updateHasData_work;

struct addDataSetToLog_work_t
{
    struct k_work work;
    uint8_t *pDataset;
    uint32_t datasetTimestamp;
} addDataSetToLog_work;

//CLI work
typedef struct __SCAN_LOG_FOR_FLAGS_T__
{
    uint8_t flags;
    uint32_t count;
} scan_log_for_flags_t;

struct cliScanLogForFlags_work_t
{
    struct k_work work;
    scan_log_for_flags_t params;

} cliScanLogForFlags_work;

struct cliAckLog_work_t
{
    struct k_work work;
    uint32_t ackAll;

} cliAckLog_work;

struct cliGetEntry_work_t
{
    struct k_work work;
    uint32_t numGet;
    struct fcb_entry loc;

} cliGetEntry_work;

struct testLogAdd_work_t
{
    struct k_work work;
    uint32_t addrOption;
    uint32_t repeat;
    int8_t rssi;
    uint8_t scanComplete;
    uint8_t clearDBSamp;
} testLogAdd_work;

K_FIFO_DEFINE(dataLogEntryAck_fifo);
#if defined(CONFIG_BOARD_BT710)
K_HEAP_DEFINE(dataLog_heap, 2432); //(16 bytes X 64 devices) + LOG_ENTRY_SIZE + (extra buffer)
#else
K_HEAP_DEFINE(dataLog_heap, 7040); //(16 bytes X 352 devices) + LOG_ENTRY_SIZE + (extra buffer)
#endif

typedef struct __EVT_ACK_FIFO_ITEM_T__
{
    void *fifo_reserved; /* 1st word reserved for use by fifo */
    uint32_t addr;

} evt_ack_fifo_item_t;

typedef struct __DATA_LOG_INFO_T__
{
    uint32_t head;                //oldest unacked entry
    uint32_t tail;                //next available entry
    uint32_t unAckedCount;        //number of unacked entries
    uint32_t recordHighWaterMark; //highest number of records in any entry

} data_log_info_t;

typedef struct
{
    uint8_t flags;
    uint8_t state; // put back to discovery state
    uint8_t serial[BT_MAC_ADDR_LEN];
    uint32_t entryAddr;               // reset to 0
    uint32_t firstSeen;               // not reset each scan
    uint32_t entryTimestamp;          // gets rewritten each scan
    uint32_t lastSeen;                // gets rewritten each scan
    int32_t rssiAccumulatedLastScan;  // gets rewritten each scan
    uint16_t rssiSampleCntLastScan;   // gets rewritten each scan
    uint16_t rssiRecordIdx;           // reset to 0
    uint16_t totalScanIntervalOffset; // reset to 0
    uint16_t scanIntervalAlarmCount;  // remains intact across datalog collections

    struct __attribute__((packed))
    {
        uint16_t scanIntervalOffset;
        int8_t rssiRunningAvg;
        int8_t txPwr;
        int8_t motion;
        uint8_t status;
    } rssiRecord[MAX_DISC_RECORDS]; // reset to 0

} device_disc_t;

typedef struct __DATA_LOG_CLI_READ_PARAMS_T__
{
    uint32_t desiredEntryNum;
    uint32_t currEntryNum;
    uint8_t readEntryOrRaw;
    uint8_t readRawLen;
    uint8_t entryFound;

} data_log_cli_read_params_t;

typedef struct __DATA_LOG_GET_NEXT_UNACK_T__
{
    uint32_t *pEntryAddr;
    uint8_t *pDataOut;
    uint16_t *pEntrySize;

} data_log_get_next_unack_t;

typedef struct __DATA_LOG_SCAN_FLAGS_T__
{
    uint8_t flags;
    uint32_t numEntries;

} data_log_scan_flags_t;

data_log_info_t dataLogInfo;
data_log_info_t downloadTempLogInfo;

struct k_sem dataLogWriteSem;
struct k_sem dataLogUpdateInProgSem;

static bool readInProgress = false;
static bool timeSetOnce = false;
static uint8_t num_entries_to_ack = 0;

static bool testAddInProgress = false;

//Index used for iterating through accel data to add to log (which occurs over multiple work items)
//should only be used in addDataSetToLog and UpdateHasDataFlagWork
static uint32_t accelDataIdx = 0;

struct fcb datalog_fcb;

/* Sectors for FCB */
struct flash_sector datalog_fcb_sector[] = {
    [0] = {.fs_off = 0, .fs_size = FCB_SECTOR_SIZE}, /* 4K */
    [1] = {.fs_off = 0x01000, .fs_size = FCB_SECTOR_SIZE},
    [2] = {.fs_off = 0x02000, .fs_size = FCB_SECTOR_SIZE},
    [3] = {.fs_off = 0x03000, .fs_size = FCB_SECTOR_SIZE},
    [4] = {.fs_off = 0x04000, .fs_size = FCB_SECTOR_SIZE},
    [5] = {.fs_off = 0x05000, .fs_size = FCB_SECTOR_SIZE},
    [6] = {.fs_off = 0x06000, .fs_size = FCB_SECTOR_SIZE},
    [7] = {.fs_off = 0x07000, .fs_size = FCB_SECTOR_SIZE},
    [8] = {.fs_off = 0x08000, .fs_size = FCB_SECTOR_SIZE},
    [9] = {.fs_off = 0x09000, .fs_size = FCB_SECTOR_SIZE},
    [10] = {.fs_off = 0x0a000, .fs_size = FCB_SECTOR_SIZE},
    [11] = {.fs_off = 0x0b000, .fs_size = FCB_SECTOR_SIZE},
    [12] = {.fs_off = 0x0c000, .fs_size = FCB_SECTOR_SIZE},
    [13] = {.fs_off = 0x0d000, .fs_size = FCB_SECTOR_SIZE},
    [14] = {.fs_off = 0x0e000, .fs_size = FCB_SECTOR_SIZE},
    [15] = {.fs_off = 0x0f000, .fs_size = FCB_SECTOR_SIZE},
    [16] = {.fs_off = 0x10000, .fs_size = FCB_SECTOR_SIZE},
    [17] = {.fs_off = 0x11000, .fs_size = FCB_SECTOR_SIZE},
    [18] = {.fs_off = 0x12000, .fs_size = FCB_SECTOR_SIZE},
    [19] = {.fs_off = 0x13000, .fs_size = FCB_SECTOR_SIZE},
    [20] = {.fs_off = 0x14000, .fs_size = FCB_SECTOR_SIZE},
    [21] = {.fs_off = 0x15000, .fs_size = FCB_SECTOR_SIZE}};

//=================================================================================================
// Local Function Prototypes
//=================================================================================================

static int datalog_fcb_walk_count_entries_cb(struct fcb_entry_ctx *entry_ctx, void *arg);
static int datalog_fcb_walk_count_bytes_cb(struct fcb_entry_ctx *entry_ctx, void *arg);
static int datalog_fcb_walk_flag_cb(struct fcb_entry_ctx *entry_ctx, void *arg);
static int datalog_fcb_walk_print_cb(struct fcb_entry_ctx *entry_ctx, void *arg);
static void UpdateHasDataFlagWork(struct k_work *work);
static void ackAllEntriesInLog(uint8_t numToAck);
static void addDataSetToLog(struct k_work *work);
static void dlog_log_interval_timer_handler(struct k_timer *dummy);
static void cliGetEntry(struct k_work *work);
static void testLogAdd(struct k_work *work);
static void cliScanLogForFlags(struct k_work *work);
static void cliAckLog(struct k_work *work);

K_TIMER_DEFINE(dlog_log_interval_timer, dlog_log_interval_timer_handler, NULL);

//=================================================================================================
// Global Function Definitions
//=================================================================================================

void DataLogInit(void)
{
    struct fcb *fcb;
    fcb = &datalog_fcb;

    // datalog_fcb.f_erase_value = 0xFF;
    // fcb->f_magic = 0x12345678;
    fcb->f_sectors = datalog_fcb_sector;
    fcb->f_sector_cnt = sizeof(datalog_fcb_sector) / sizeof(struct flash_sector);

    int rc = fcb_init(FCB_FLASH_AREA_ID, fcb);
    if (rc != 0)
    {
        LOG_ERR("fcb init failed %d", rc);
    }

    k_work_init(&updateHasData_work, UpdateHasDataFlagWork);
    k_work_init(&addDataSetToLog_work.work, addDataSetToLog);

    k_work_init(&cliScanLogForFlags_work.work, cliScanLogForFlags);
    k_work_init(&cliGetEntry_work.work, cliGetEntry);
    k_work_init(&testLogAdd_work.work, testLogAdd);
    k_work_init(&cliAckLog_work.work, cliAckLog);

    k_sem_init(&dataLogWriteSem, 1, 1);
    k_sem_init(&dataLogUpdateInProgSem, 1, 1);

    int data = 0;
    for (int i = 0; i < MAX_SAMPLE_CNT; i++)
    {
        testAccelData[i].x = data++;
        testAccelData[i].y = data++;
        testAccelData[i].z = data++;
    }

    /* Update Has Data flag after recovering tracked devices */
    UpdateHasDataFlag();
}

bool DataLogStartDataCollectionCb(void)
{
    // take semaphore to prevent being collected while adding new data
    //(associated k_sem_give is called by addDataSetToLog)
    int32_t rc = k_sem_take(&dataLogUpdateInProgSem, K_MSEC(LOG_WRITE_SEM_TIMEOUT));
    if (rc)
    {
        LOG_ERR("DataLogStartDataCollectionCb error, sem timeout %d", rc);
        if(!isBleConnected())
        {
            LOG_ERR("Not connected");
        }
        
        return false;
    }
    LOG_DBG("Update sem taken %d", rc);

    return true;
}

void DataLogAddDataSet(uint8_t *pData, uint32_t timestampSec)
{
    addDataSetToLog_work.pDataset = pData;
    addDataSetToLog_work.datasetTimestamp = timestampSec;
    //if battery is good and device was scanning, can write data to log
    k_work_submit(&addDataSetToLog_work.work);
}

uint32_t DataLogStartDownload()
{
    // take semaphore to prevent returning if there are pending acks
    // also prevent adding if getlogsize is requested.
    //(associated k_sem_give is called by DataLogReadComplete or DataLogReadAbort)
    int32_t rc = k_sem_take(&dataLogUpdateInProgSem, K_MSEC(LOG_WRITE_SEM_TIMEOUT));
    if (rc)
    {
        LOG_ERR("DataLogGetLogSize error, sem timeout");
        return 0;
    }
    LOG_DBG("Update sem taken");

    uint32_t size = DataLogGetLogSize();

    if (size != 0)
    {
        //Set temporary pointers to head and tail
        downloadTempLogInfo.head = dataLogInfo.head;
        downloadTempLogInfo.tail = dataLogInfo.tail;
        downloadTempLogInfo.unAckedCount = dataLogInfo.unAckedCount;
        downloadTempLogInfo.recordHighWaterMark = dataLogInfo.recordHighWaterMark;

        readInProgress = true;
    }
    else
    {
        //Give semaphore back to allow device to start recording data again
        k_sem_give(&dataLogUpdateInProgSem);
        LOG_DBG("Update sem given (no log to download)");
    }

    num_entries_to_ack = 0;

    return size;
}

void DataLogResetDownload(void)
{
    //Discard any items in the Ack fifo
    int32_t ret = k_fifo_is_empty(&dataLogEntryAck_fifo);
    if (ret == 0)
    {
        evt_ack_fifo_item_t *pEvtToAck;
        uint8_t cnt = 0;

        //Get event from queue
        pEvtToAck = k_fifo_get(&dataLogEntryAck_fifo, K_NO_WAIT);
        while (pEvtToAck != NULL)
        {
            k_heap_free(&dataLog_heap, pEvtToAck);
            cnt++;
            pEvtToAck = k_fifo_get(&dataLogEntryAck_fifo, K_NO_WAIT);
        }
        // LOG_WRN("%d events discarded from ack fifo", cnt);
    }
    else
    {
        // LOG_DBG("No events in ack fifo");
    }

    //Set temporary pointers to head and tail
    downloadTempLogInfo.head = dataLogInfo.head;
    downloadTempLogInfo.tail = dataLogInfo.tail;
    downloadTempLogInfo.unAckedCount = dataLogInfo.unAckedCount;
    downloadTempLogInfo.recordHighWaterMark = dataLogInfo.recordHighWaterMark;

    LOG_DBG("Log Download reset.");
}

void DataLogAckEntries(uint32_t forceAll)
{
    if(forceAll)
    {
        num_entries_to_ack = DATASET_NUM_ENTRIES;
    }
 
    LOG_DBG("DataLogAckEntries. %d %d", forceAll, num_entries_to_ack);   

    //Ack all remaining
    if (num_entries_to_ack > 0)
    {
        //Call function that will ack all entries in the fifo before returning
        ackAllEntriesInLog(num_entries_to_ack);
    }
    else
    {
        LOG_DBG("No events to ack");
    }
    
    LOG_INF("%d entries acked and deleted.", num_entries_to_ack);
    num_entries_to_ack = 0;
}

uint32_t DataLogGetNextEntry(struct fcb_entry *pPrevEntryLoc, uint8_t *pDataOut, uint16_t *pEntrySize, uint32_t available_len)
{
    int32_t rc;
    log_entry_t *pEntry; // necessary to check if an entry is acked or not
    struct fcb *fcb;
    fcb = &datalog_fcb;

    if (pPrevEntryLoc == NULL || pDataOut == NULL || pEntrySize == NULL || available_len == 0)
        return -1; /* params must not be NULL */

    if (!readInProgress)
    {
        //Set up read
        LOG_DBG("Incorrect state for log download. Setup now");
        uint32_t size = DataLogStartDownload();
        if (size == 0)
        {
            return -1;
        }
    }

    int ret = 0;
    struct fcb_entry prevEntry = *pPrevEntryLoc;
    
    if (available_len >= LOG_ENTRY_SIZE)
    {
        do
        {
            if (prevEntry.fe_sector == NULL)
            {
                LOG_DBG("Get next after oldest 0x%x, elem %d", LOG_ADDR_START, prevEntry.fe_elem_off);
            }
            else
            {
                LOG_DBG("Get next after 0x%x, elem %d", LOG_ADDR_START + FCB_ENTRY_FA_DATA_OFF(prevEntry), prevEntry.fe_elem_off);
            }

            ret = fcb_getnext(fcb, &prevEntry);
            if (!ret)
            {
                rc = flash_area_read(fcb->fap,
                                     FCB_ENTRY_FA_DATA_OFF(prevEntry),
                                     pDataOut, LOG_ENTRY_SIZE);

                pEntry = (log_entry_t *)pDataOut;

                if ((pEntry->header.flags & LOG_ENTRY_FLAGS_UNACK) == LOG_ENTRY_FLAGS_UNACK)
                {
                    //Only increment this count only after 1st call since only subsequent calls to this function indicate an 'ack' of the previous packet
                    num_entries_to_ack++;

                    *pPrevEntryLoc = prevEntry;
                    *pEntrySize = LOG_ENTRY_SIZE;
                    pEntry->header.entrySize = *pEntrySize; /* store the entry size in the entry header reserved bytes */
                    LOG_DBG("Found unacked 0x%x, elem %d, %d %d", LOG_ADDR_START + FCB_ENTRY_FA_DATA_OFF(prevEntry), prevEntry.fe_elem_off, pEntry->header.entrySize, num_entries_to_ack);
                    break;
                }
                else
                {
                    /* in this case, if we find an empty/ack'ed record, just continue to the next one. prevEntry should have been updated at this point */
                    LOG_DBG("Found acked or empty %x @ 0x%x, elem %d", pEntry->header.flags, LOG_ADDR_START + FCB_ENTRY_FA_DATA_OFF(prevEntry), prevEntry.fe_elem_off);
                }
            }
            else
            {
                //fcb_getnext returns -ENOTSUP at end of FCB
                if (ret == -ENOTSUP)
                {
                    LOG_DBG("End of FCB reached.");
                }
                else
                {
                    LOG_ERR("fcb_getnext failed %d", ret);
                }
                (*pPrevEntryLoc).fe_sector = NULL; //clear entry so that subsequent fcb_getnext returns oldest
                break;
            }
        } while (ret == 0);
    }
    else
    {
        //no more space to fit this entry into given buffer
        
        /* Return error since this entry won't fit in the provided buffer. 
        * Keep pEntrySize as non-0 to indicate there
        * was a valid unack'ed entry */
        *pEntrySize = LOG_ENTRY_SIZE;

        return -ENOSPC;
    }

    return 0;
}

static uint32_t DataLogGetNumEntries(void)
{
    uint32_t totalEntries;
    scan_log_for_flags_t scanParams;
    scanParams.flags = LOG_ENTRY_FLAGS_UNACK;
    scanParams.count = 0;
    struct fcb *fcb;
    fcb = &datalog_fcb;

    //Walk over entire storage to count number of entries
    uint32_t rc = fcb_walk(fcb, 0, datalog_fcb_walk_count_entries_cb, &totalEntries);
    if (rc)
    {
        LOG_ERR("Count all entries failed %d", rc);
    }

    rc = fcb_walk(fcb, 0, datalog_fcb_walk_flag_cb, &scanParams);
    if (rc)
    {
        LOG_ERR("Count unacked entries failed %d", rc);
    }
    else
    {
        LOG_DBG("Found %d unacked entries", scanParams.count);
    }

    return scanParams.count;
}

uint32_t DataLogGetLogSize(void)
{
    uint32_t log_total_bytes = 0;

    struct fcb *fcb;
    fcb = &datalog_fcb;

    //Walk over entire storage to count number of entries
    uint32_t rc = fcb_walk(fcb, 0, datalog_fcb_walk_count_bytes_cb, &log_total_bytes);
    if (rc)
    {
        LOG_ERR("DataLogGetNumEntries failed %d", rc);
    }

    return log_total_bytes;
}

uint32_t DataLogGetUnackedEntryCount(void)
{
    return DataLogGetNumEntries();
}

uint32_t DataLogGetDownloadEntryCount(void)
{
    return DataLogGetNumEntries();
}

void DataLogReadComplete(void)
{
    if (readInProgress)
    {
        //End read
        readInProgress = false;
    }

    DataLogAckEntries(false);

    UpdateHasDataFlag();

    //Clear download pointers
    downloadTempLogInfo.head = 0;
    downloadTempLogInfo.tail = 0;
    downloadTempLogInfo.unAckedCount = 0;
    downloadTempLogInfo.recordHighWaterMark = 0;

    LOG_DBG("Read completed.");

    //Give semaphore back to allow device to start recording data again
    k_sem_give(&dataLogUpdateInProgSem);
    LOG_DBG("Update sem given");
}

void DataLogReadAbort(void)
{
    if (readInProgress)
    {
        //End read
        readInProgress = false;
        LOG_DBG("Read aborted.");
    }

    //Discard any items in the Ack fifo - they were not successfully sent.
    int32_t ret = k_fifo_is_empty(&dataLogEntryAck_fifo);
    if (ret == 0)
    {
        evt_ack_fifo_item_t *pEvtToAck;
        uint8_t cnt = 0;

        //Get event from queue
        pEvtToAck = k_fifo_get(&dataLogEntryAck_fifo, K_NO_WAIT);
        while (pEvtToAck != NULL)
        {
            k_heap_free(&dataLog_heap, pEvtToAck);
            cnt++;
            pEvtToAck = k_fifo_get(&dataLogEntryAck_fifo, K_NO_WAIT);
        }
        LOG_WRN("%d events discarded from ack fifo", cnt);
    }
    else
    {
        LOG_DBG("No events in ack fifo");
    }

    UpdateHasDataFlag();

    //Clear download pointers
    downloadTempLogInfo.head = 0;
    downloadTempLogInfo.tail = 0;
    downloadTempLogInfo.unAckedCount = 0;
    downloadTempLogInfo.recordHighWaterMark = 0;

    //Update high water mark after abort since high water mark may have changed
    dataLogInfo.recordHighWaterMark = 0;

    //Give semaphore back to allow device to start recording data again
    k_sem_give(&dataLogUpdateInProgSem);
    LOG_DBG("Update sem given");
}

void DataLogWasRead(void)
{
    clear_ble_ad_flag_bit(ADV_FLAGS_HAS_LOG_DATA);

    uint16_t interval = get_log_interval();

    //If interval is 0, handler will fire immediately and update the HAS DATA flag
    uint32_t periodS = interval * 60;
    k_timer_start(&dlog_log_interval_timer, K_SECONDS(periodS), K_MSEC(0));
}

void DataLogErase(void)
{
    int32_t rc = k_sem_take(&dataLogWriteSem, K_MSEC(LOG_WRITE_SEM_TIMEOUT));
    if (rc)
    {
        LOG_ERR("DataLogErase failed, sem timeout");
        return;
    }

    //Erase oldest
    struct fcb *fcb;
    fcb = &datalog_fcb;
    rc = fcb_rotate(fcb);
    if(rc != 0)
    {
        LOG_ERR("FCB erase/rotate failed, %d", rc);
    }

    k_sem_give(&dataLogWriteSem);
}

void DataLogEraseAll(void)
{
    struct fcb *fcb;
    fcb = &datalog_fcb;
    fcb_clear(fcb);
}

void DataLogStopLogIntervalTimer(void)
{
    k_timer_stop(&dlog_log_interval_timer);
    UpdateHasDataFlag();
}

void UpdateHasDataFlag(void)
{
    k_work_submit(&updateHasData_work);
}

void DataLogTimeSetCb(uint32_t oldTimeSec, uint32_t newTimeSec)
{
    //Unused at the moment. Flag exists to allow performing of some actions on first time clock is set
    if (!timeSetOnce)
    {
        timeSetOnce = true;
    }
}

void DataLogDisconnectDb(void)
{
    //Always give semaphore back on disconnect to allow device to start recording data again
    k_sem_give(&dataLogUpdateInProgSem);
    LOG_DBG("Update sem given");

    DataLogReadAbort(); //clear ACK list
}

void DataLogAddTestEntry(uint32_t numRecords)
{
    testLogAdd_work.addrOption = 0;
    testLogAdd_work.repeat = numRecords;
    testLogAdd_work.rssi = -99; //default to be out of alarm range (otherwise could alarm on every add)
    testLogAdd_work.scanComplete = 1;
    testLogAdd_work.clearDBSamp = 1;

    if ((!testAddInProgress) && (testLogAdd_work.repeat > 0))
    {
        LOG_INF("writing %d records (addrOption: %d)", testLogAdd_work.repeat, testLogAdd_work.addrOption);
        k_work_submit(&testLogAdd_work.work);
    }
}

uint16_t get_log_interval()
{
    return DLOG_LOG_INTERVAL_MIN_DEF;
}

void DataLogDataCollectCompleteCb(void)
{
    k_sem_give(&dataLogUpdateInProgSem);
}

//=================================================================================================
// Local Function Definitions
//=================================================================================================

static int datalog_fcb_walk_count_entries_cb(struct fcb_entry_ctx *entry_ctx, void *arg)
{
    uint16_t len;
    uint8_t test_data[LOG_ENTRY_SIZE];
    int rc;
    int *var_cnt = (int *)arg;

    len = entry_ctx->loc.fe_data_len;

    rc = flash_area_read(entry_ctx->fap,
                         FCB_ENTRY_FA_DATA_OFF(entry_ctx->loc),
                         test_data, len);
    if (rc != 0)
    {
        LOG_ERR("read call failure %d", rc);
    }

    (*var_cnt)++;

    return 0; //returning non-zero value will end the "walk" early
}

static int datalog_fcb_walk_count_bytes_cb(struct fcb_entry_ctx *entry_ctx, void *arg)
{
    uint16_t len;
    uint8_t test_data[LOG_ENTRY_SIZE];
    int rc;
    int *num_bytes = (int *)arg;

    len = entry_ctx->loc.fe_data_len;

    rc = flash_area_read(entry_ctx->fap,
                         FCB_ENTRY_FA_DATA_OFF(entry_ctx->loc),
                         test_data, len);
    if (rc != 0)
    {
        LOG_ERR("read call failure %d", rc);
    }

    (*num_bytes) += len;

    return 0; //returning non-zero value will end the "walk" early
}

static int datalog_fcb_walk_flag_cb(struct fcb_entry_ctx *entry_ctx, void *arg)
{
    uint16_t len;
    log_entry_t tmp_entry;
    int rc;
    scan_log_for_flags_t *pParams = (scan_log_for_flags_t *)arg;

    len = entry_ctx->loc.fe_data_len;

    rc = flash_area_read(entry_ctx->fap,
                         FCB_ENTRY_FA_DATA_OFF(entry_ctx->loc),
                         (uint8_t *)&tmp_entry, len);
    if (rc != 0)
    {
        LOG_ERR("read call failure %d", rc);
    }

    if ((pParams->flags & tmp_entry.header.flags) == pParams->flags)
    {
        pParams->count++;
    }

    return 0; //returning non-zero value will end the "walk" early
}

static int datalog_fcb_walk_print_cb(struct fcb_entry_ctx *entry_ctx, void *arg)
{
    uint16_t len;
    uint8_t readEntrybytes[LOG_ENTRY_SIZE];
    int rc;
    data_log_cli_read_params_t *pParams = (data_log_cli_read_params_t *)arg;

    len = entry_ctx->loc.fe_data_len;

    //Only display 1 entry, desired entry number (0 = oldest)
    if (pParams->desiredEntryNum == pParams->currEntryNum)
    {
        LOG_INF("Entry address 0x%x", FCB_ENTRY_FA_DATA_OFF(entry_ctx->loc));
        rc = flash_area_read(entry_ctx->fap,
                             FCB_ENTRY_FA_DATA_OFF(entry_ctx->loc),
                             readEntrybytes, len);

        if (rc == 0)
        {
            pParams->entryFound = 1;

            if (pParams->readEntryOrRaw == 0)
            {
                log_entry_t *readEntry = (log_entry_t *)readEntrybytes;
                if (readEntry->header.entryStart == LOG_ENTRY_START_BYTE)
                {
                    LOG_INF("readEntry->header.entryStart = %x", readEntry->header.entryStart);
                    LOG_INF("readEntry->header.flags = %x", readEntry->header.flags);
                    LOG_INF("readEntry->header.sampleCount = %x", readEntry->header.sampleCount);
                    LOG_INF("readEntry->header.serial = %02x%02x%02x%02x%02x%02x",
                            readEntry->header.serial[0],
                            readEntry->header.serial[1],
                            readEntry->header.serial[2],
                            readEntry->header.serial[3],
                            readEntry->header.serial[4],
                            readEntry->header.serial[5]);
                    LOG_INF("readEntry->header.timestamp = %d", readEntry->header.timestamp);
                    LOG_INF("readEntry->header.entrySize = %d", readEntry->header.entrySize);

                    if (readEntry->data.recordType == DLOG_RECORD_TYPE_AD_RSSI_TRACKING)
                    {
                        LOG_INF("readEntry->data.rssi[0] = %d", readEntry->data.rssiTrackData[0].rssi);
                    }
                    else if (readEntry->data.recordType == DLOG_RECORD_TYPE_AD_RSSI_TRACKING_W_TS)
                    {
                        LOG_INF("readEntry->data.scanIntOff[0] = %d", readEntry->data.rssiTrackWTsData[0].scanIntervalOffset);
                        LOG_INF("readEntry->data.rssi[0] = %d", readEntry->data.rssiTrackWTsData[0].rssi);
                    }
                    else if (readEntry->data.recordType == DLOG_RECORD_TYPE_ACCEL)
                    {
                        LOG_INF("readEntry->data.accelData[0].x = %d", readEntry->data.accelData[0].xyz.x);
                        LOG_INF("readEntry->data.accelData[0].y = %d", readEntry->data.accelData[0].xyz.y);
                        LOG_INF("readEntry->data.accelData[0].z = %d", readEntry->data.accelData[0].xyz.z);
                    }
                    LOG_INF("entrySize = %d", len);
                }
                else
                {
                    LOG_INF("Empty entry");
                    LOG_INF("readEntry->header.entryStart = %x", readEntry->header.entryStart);
                    LOG_INF("readEntry->header.flags = %x", readEntry->header.flags);
                }
            }
            else
            {
                for (uint32_t i = 0; i < pParams->readRawLen; i++)
                {
                    printk("%02X", readEntrybytes[i]);
                }
            }
        }
        else
        {
            LOG_ERR("Read failed %x", rc);
        }
    }

    pParams->currEntryNum++;

    return pParams->entryFound; //returning non-zero value will end the "walk" early. this will keep reading until entry is found
}

static void ackAllEntriesInLog(uint8_t numToAck)
{
    int32_t rc = k_sem_take(&dataLogWriteSem, K_MSEC(LOG_WRITE_SEM_TIMEOUT));
    if (rc)
    {
        LOG_ERR("ackAllEntriesInLog failed, sem timeout");
        return;
    }

    struct fcb *fcb;
    fcb = &datalog_fcb;

    struct fcb_entry loc;
    log_entry_t entry;
    memset((uint8_t *)&loc, 0, sizeof(loc));

    do
    {
        numToAck--;

        rc = fcb_getnext(fcb, &loc);
        if (!rc)
        {
            rc = flash_area_read(fcb->fap,
                                 FCB_ENTRY_FA_DATA_OFF(loc),
                                 (uint8_t *)&entry, LOG_ENTRY_SIZE);

            if (rc == 0)
            {
                entry.header.flags &= ~LOG_ENTRY_FLAGS_UNACK;

                rc = flash_area_write(fcb->fap, FCB_ENTRY_FA_DATA_OFF(loc),
                                      (uint8_t *)&entry, sizeof(uint32_t)); //only write 1st 4 bytes to reduce overwriting of flash location (start byte, flags, interval)
                if (rc)
                {
                    LOG_ERR("flash_area_write failed %d", rc);
                }
                else
                {
                    LOG_DBG("Acked entry 0x%x", LOG_ADDR_START + FCB_ENTRY_FA_DATA_OFF(loc));
                }
            }
        }
        else
        {
            //fcb_getnext returns -ENOTSUP at end of FCB
            if (rc == -ENOTSUP)
            {
                LOG_DBG("End of FCB reached.");
            }
            else
            {
                LOG_ERR("fcb_getnext failed %d", rc);
            }
            break;
        }
    } while (numToAck);

    k_sem_give(&dataLogWriteSem);
}


static void UpdateHasDataFlagWork(struct k_work *work)
{
    //Only update flag if timer has expired and not currently processing device database after a scan
    if ((k_timer_remaining_get(&dlog_log_interval_timer) == 0) && (accelDataIdx == 0))
    {
        //Update advertising flag (based on record high water mark and unack'ed expired entries)
        uint16_t threshold = HAS_DATA_THRESHOLD_DEF;
        uint32_t numEntries = DataLogGetNumEntries();

        if (numEntries >= threshold)
        {
            set_ble_ad_flag_bit(ADV_FLAGS_HAS_LOG_DATA);
            LOG_DBG("Log timer exp-HAS_DATA=1 (high water)");
        }
        else
        {
            clear_ble_ad_flag_bit(ADV_FLAGS_HAS_LOG_DATA);
            LOG_DBG("Log timer exp-HAS_DATA=0");
        }
    }
    else
    {
        // LOG_DBG("LogIntvl timer run-HAS_DATA force 0");
        clear_ble_ad_flag_bit(ADV_FLAGS_HAS_LOG_DATA);
    }
}

static void addDataSetToLog(struct k_work *work)
{
    struct addDataSetToLog_work_t *params =
        CONTAINER_OF(work, struct addDataSetToLog_work_t, work);

    log_entry_t entry;
    static uint16_t numAdded = 0;

    int32_t rc = k_sem_take(&dataLogWriteSem, K_MSEC(LOG_WRITE_SEM_TIMEOUT));
    if (rc)
    {
        LOG_ERR("addDataSetToLog failed, sem timeout");
        k_sem_give(&dataLogUpdateInProgSem);
        LOG_DBG("Update sem given");
        return;
    }
    LOG_DBG("dataLogWriteSem taken");

    uint8_t recordType = DLOG_RECORD_TYPE_DEFAULT;
    static uint16_t entryCount = 0;//keep static variable to increment over a single dataset

    //new device entry - fill in header and data
    bt_addr_le_t addr[CONFIG_BT_ID_MAX];
    //use test addr if bt is disabled
    addr[0].a.val[0] = 0xAA;
    addr[0].a.val[1] = 0xBB;
    addr[0].a.val[2] = 0xCC;
    addr[0].a.val[3] = 0xDD;
    addr[0].a.val[4] = 0xEE;
    addr[0].a.val[5] = 0xFF;
    get_bt_address(addr);

    SysTime_t time;
    if (params->datasetTimestamp == 0)
    {
        time = SysTimeGet();
    }
    else
    {
        time.Seconds = params->datasetTimestamp;
    }

    entry.header.entryStart = LOG_ENTRY_START_BYTE;
    entry.header.flags = 0xFF; //set default
    entry.header.flags |= LOG_ENTRY_FLAGS_UNACK;
    if (is_set_ble_ad_flag_bit(ADV_FLAGS_HAS_EPOCH_TIME))
        entry.header.flags &= ~LOG_ENTRY_FLAGS_TIME_INVALID; //clear bit to indicate time is valid
    entry.header.sampleCount = entryCount++;
    memcpy(entry.header.serial, addr[0].a.val, sizeof(addr[0].a.val));
    entry.header.timestamp = params->datasetTimestamp;
    entry.header.entrySize = LOG_ENTRY_SIZE; //always write full entry, though if there are not enough samples to fill entry, there will just be FFs at the end.

    uint16_t numSamples = LOG_ENTRY_MAX_NUM_RECORDS;
    if ((MAX_SAMPLE_CNT - (accelDataIdx/sizeof(sample_t))) < LOG_ENTRY_MAX_NUM_RECORDS)
    {
        numSamples = MAX_SAMPLE_CNT - (accelDataIdx/sizeof(sample_t));
    }

    if (numSamples == 0)
    {
        LOG_ERR("numSamples is 0");
    }
    else
    {
        memset(&entry.data.accelData[0], 0xFF, sizeof(entry.data.accelData));

        LOG_DBG("numSamples %d, accelDataIdx %d out of %d", numSamples, accelDataIdx, MAX_SAMPLE_CNT*sizeof(sample_t));
        for (int i = 0; i < numSamples; i++)
        {
            entry.data.accelData[i].recordType = recordType;            
            memcpy(&entry.data.accelData[i].xyz, &addDataSetToLog_work.pDataset[accelDataIdx], sizeof(sample_t));
            accelDataIdx += sizeof(sample_t);
        }

        struct fcb_entry loc;
        struct fcb *fcb;
        fcb = &datalog_fcb;
        int32_t ret = fcb_append(fcb, LOG_ENTRY_SIZE, &loc);
        if (ret == -ENOSPC)
        {
            LOG_DBG("log full. erase oldest");
            ret = fcb_rotate(fcb);
            if (ret)
            {
                LOG_ERR("fcb_rotate failed %d", ret);
            }
            ret = fcb_append(fcb, LOG_ENTRY_SIZE, &loc);

            if (ret)
            {
                LOG_ERR("fcb_append after rotate failed %d", ret);
            }
        }
        else if (ret)
        {
            LOG_ERR("fcb_append failed %d", ret);
        }

        if (ret == 0) //append succeeded
        {
            LOG_DBG("flash_write (0x%x, 0x%x)", loc.fe_sector->fs_off, FCB_ENTRY_FA_DATA_OFF(loc));
            ret = flash_area_write(fcb->fap, FCB_ENTRY_FA_DATA_OFF(loc),
                                   (uint8_t *)&entry, LOG_ENTRY_SIZE);
            if (ret)
            {
                LOG_ERR("flash_area_write failed %d", ret);
            }
            ret = fcb_append_finish(fcb, &loc);
            if (ret)
            {
                LOG_ERR("fcb_append_finish failed %d", ret);
            }
        }

        if (ret == 0)
        {
            dataLogInfo.unAckedCount++;
            LOG_DBG("flash_write success! (0x%x, 0x%x, 0x%x, 0x%x, %d)", loc.fe_sector->fs_off, FCB_ENTRY_FA_DATA_OFF(loc), loc.fe_data_off, loc.fe_elem_off, loc.fe_data_len);
            k_sleep(K_MSEC(1));
            numAdded++;
        }
        else
        {
            LOG_ERR("flash_write failed! (%d, 0x%x, 0x%x, 0x%x, 0x%x, %d)", ret, loc.fe_sector->fs_off, FCB_ENTRY_FA_DATA_OFF(loc), loc.fe_data_off, loc.fe_elem_off, loc.fe_data_len);
        }
    }

    k_sem_give(&dataLogWriteSem);
    LOG_DBG("dataLogWriteSem give %d %d", accelDataIdx, entryCount);

    if (accelDataIdx < (MAX_SAMPLE_CNT*sizeof(sample_t)))
    {
        k_sleep(K_MSEC(1));
        //Re-submit work queue item to continue processing
        k_work_submit(&addDataSetToLog_work.work);
    }
    else
    {
        //Finished processing all devices from previous scan
        accelDataIdx = 0;
        entryCount = 0;

        //Give semaphore back to allow device to be collected
        k_sem_give(&dataLogUpdateInProgSem);
        LOG_DBG("Update sem given");
        LOG_INF("Dataset added. %d", numAdded);
        numAdded = 0;
        params->datasetTimestamp = 0;

        //Update advertising flags after (possibly) adding new records
        UpdateHasDataFlag();
    }

    if (testAddInProgress)
    {
        k_work_submit(&testLogAdd_work.work);
    }
}

/*
 * Allow Has Data flag in advertisement to be updated again
 */
static void dlog_log_interval_timer_handler(struct k_timer *dummy)
{
    UpdateHasDataFlag();
}

static void cliGetEntry(struct k_work *work)
{
    struct cliGetEntry_work_t *params =
        CONTAINER_OF(work, struct cliGetEntry_work_t, work);

    uint16_t entry_size = 0;
    uint8_t pEntryData[LOG_ENTRY_SIZE];
    uint32_t ret = DataLogGetNextEntry(&params->loc, pEntryData, &entry_size, LOG_ENTRY_SIZE);

    if (0 == ret)
    {
        LOG_INF("GetEntry ret 0x%x (%d)", LOG_ADDR_START + FCB_ENTRY_FA_DATA_OFF(params->loc), params->numGet);

        params->numGet--;
        if (params->numGet)
        {
            k_sleep(K_MSEC(200));
            k_work_submit(&cliGetEntry_work.work);
        }
        else
        {
            LOG_INF("GetEntry completed (%d)", params->numGet);
        }
    }
    else
    {
        LOG_INF("GetEntry err %d ret 0x%x (%d)", ret, LOG_ADDR_START + FCB_ENTRY_FA_DATA_OFF(params->loc), params->numGet);
    }
}

static void testLogAdd(struct k_work *work)
{
    /* Use "bt" CLI to stop bluetooth operations if desired (so they don't interfere with log updates from this command) */

    DataLogAddDataSet((uint8_t *)&testAccelData, SysTimeGet().Seconds);
    LOG_INF("Add test entries initiated...");

    /* Use "bt" CLI to restart bluetooth operations if desired (so they don't interfere with log updates from this command) */
}

static void cliScanLogForFlags(struct k_work *work)
{
    struct cliScanLogForFlags_work_t *workParams =
        CONTAINER_OF(work, struct cliScanLogForFlags_work_t, work);
    LOG_INF("Flags to scan for %x", workParams->params.flags);

    struct fcb *fcb;
    fcb = &datalog_fcb;

    if (workParams->params.flags > 0)
    {
        //Walk over entire storage to count number of entries
        int32_t rc = fcb_walk(fcb, 0, datalog_fcb_walk_flag_cb, &workParams->params);
        if (rc)
        {
            LOG_ERR("fcb_walk failed %d", rc);
        }
        LOG_INF("Scan complete. %d entries with flags %x found.", workParams->params.count, workParams->params.flags);
    }
    else
    {
        LOG_INF("No flags specified");
    }
}

static void cliAckLog(struct k_work *work)
{    
    struct cliAckLog_work_t *workParams =
        CONTAINER_OF(work, struct cliAckLog_work_t, work);

    DataLogAckEntries(workParams->ackAll);
    UpdateHasDataFlag();
}
/*
 * Shell nv list command
 */
static int32_t cmd_log_erase(const struct shell *shell, size_t argc, char **argv)
{
    struct fcb *fcb;
    fcb = &datalog_fcb;
    fcb_clear(fcb);

    LOG_INF("Datalog erased.");

    return 0;
}

static int32_t cmd_log_add(const struct shell *shell, size_t argc, char **argv)
{
    char *p;
    testLogAdd_work.addrOption = 1;
    testLogAdd_work.repeat = 1;
    testLogAdd_work.rssi = -99; //default to be out of alarm range (otherwise could alarm on every add)
    testLogAdd_work.scanComplete = 1;
    testLogAdd_work.clearDBSamp = 1;

    if (argc > 1)
    {
        testLogAdd_work.addrOption = (uint32_t)strtoul(argv[1], &p, 0);
    }
    if (argc > 2)
    {
        testLogAdd_work.repeat = (uint32_t)strtoul(argv[2], &p, 0);
    }
    if (argc > 3)
    {
        testLogAdd_work.rssi = (int8_t)strtol(argv[3], &p, 0);
    }
    if (argc > 4)
    {
        testLogAdd_work.scanComplete = (uint8_t)strtoul(argv[4], &p, 0);
    }
    if (argc > 5)
    {
        testLogAdd_work.clearDBSamp = (uint8_t)strtoul(argv[5], &p, 0);
    }

    if ((!testAddInProgress) && (testLogAdd_work.repeat > 0))
    {
        LOG_INF("writing %d records (addrOption: %d, rssi: %d)", testLogAdd_work.repeat, testLogAdd_work.addrOption, testLogAdd_work.rssi);
        k_work_submit(&testLogAdd_work.work);
    }

    return 0;
}

static int32_t cmd_log_read(const struct shell *shell, size_t argc, char **argv)
{
    char *p;
    data_log_cli_read_params_t params;
    
    if (argc > 1)
    {
        params.desiredEntryNum = (uint32_t)strtoul(argv[1], &p, 0);
    }
    else
    {
        params.desiredEntryNum = 0;
    }

    if (argc > 2)
    {
        params.readEntryOrRaw = (uint32_t)strtoul(argv[2], &p, 0);
    }
    else
    {
        params.readEntryOrRaw = 0;
    }

    if (argc > 3)
    {
        params.readRawLen = (uint32_t)strtoul(argv[3], &p, 0);
    }
    else
    {
        params.readRawLen = 0;
    }

    params.currEntryNum = 0;
    params.entryFound = 0; //use as a flag to indicate whether an entry was read or not

    struct fcb *fcb;
    fcb = &datalog_fcb;

    fcb_walk(fcb, 0, datalog_fcb_walk_print_cb, (uint8_t *)&params);
    if (params.entryFound == 0)
    {
        LOG_INF("Entry not in log (%d)", params.desiredEntryNum);
    }

    return 0;
}

static int32_t cmd_log_getnext(const struct shell *shell, size_t argc, char **argv)
{
    char *p;
    cliGetEntry_work.numGet = 1;

    if (argc > 1)
    {
        cliGetEntry_work.numGet = (uint32_t)strtoul(argv[1], &p, 0);
    }

    bool start = true;
    if (!readInProgress)
    {
        //auto start download
        uint32_t size = DataLogStartDownload();

        if (size == 0)
        {
            start = false;
        }
        else
        {
            LOG_INF("Start download: %d", size);
        }
    }

    if (start)
    {
        (void)memset(&cliGetEntry_work.loc, 0, sizeof(cliGetEntry_work.loc)); //set entry to 0 to get oldest entry
        num_entries_to_ack = 0;

        k_work_submit(&cliGetEntry_work.work);
    }
    else
    {
        LOG_INF("Not available or log size is 0");
    }

    //user must manually ack when desired (or with "dlog complete")

    return 0;
}

static int32_t cmd_log_complete(const struct shell *shell, size_t argc, char **argv)
{
    char *p;
    uint32_t completeOrAbort = 0;
    uint32_t callWasRead = 0;
    if (argc > 1)
    {
        completeOrAbort = (uint32_t)strtoul(argv[1], &p, 0);
    }

    if (argc > 2)
    {
        callWasRead = (uint32_t)strtoul(argv[2], &p, 0);
    }

    if (!completeOrAbort)
    {
        DataLogReadComplete();
    }
    else
    {
        DataLogReadAbort();
    }

    if (callWasRead)
    {
        DataLogWasRead();
    }

    return 0;
}

static int32_t cmd_log_ack(const struct shell *shell, size_t argc, char **argv)
{
    char *p;
    uint32_t all = 0;
    if (argc > 1)
    {
        all = (uint32_t)strtoul(argv[1], &p, 0);
    }

    cliAckLog_work.ackAll = all;
    k_work_submit(&cliAckLog_work.work);

    LOG_INF("ACK cli complete");

    return 0;
}

static int32_t cmd_log_info(const struct shell *shell, size_t argc, char **argv)
{
    LOG_INF("fcb f_version = 0x%x", datalog_fcb.f_version);
    LOG_INF("fcb f_sector_cnt = 0x%x", datalog_fcb.f_sector_cnt);
    LOG_INF("fcb f_scratch_cnt = 0x%x", datalog_fcb.f_scratch_cnt);
    LOG_INF("fcb free sectors = %d", fcb_free_sector_cnt(&datalog_fcb));
    LOG_INF("");
    LOG_INF("fcb f_oldest.fs_off = 0x%x", datalog_fcb.f_oldest->fs_off);
    LOG_INF("fcb f_oldest.fs_size = 0x%x", datalog_fcb.f_oldest->fs_size);
    LOG_INF("");
    LOG_INF("fcb f_active:");
    LOG_INF("fcb f_active.fe_sector.fs_off = 0x%x", datalog_fcb.f_active.fe_sector->fs_off);
    LOG_INF("fcb f_active.fe_sector.fs_size = 0x%x", datalog_fcb.f_active.fe_sector->fs_size);
    LOG_INF("fcb f_active.fe_elem_off = 0x%x", datalog_fcb.f_active.fe_elem_off);
    LOG_INF("fcb f_active.fe_data_off = 0x%x", datalog_fcb.f_active.fe_data_off);
    LOG_INF("fcb f_active.fe_data_len = 0x%x", datalog_fcb.f_active.fe_data_len);
    LOG_INF("fcb f_active_id = 0x%x", datalog_fcb.f_active_id);
    LOG_INF("");
    LOG_INF("fcb f_align = 0x%x", datalog_fcb.f_align);
    LOG_INF("");
    LOG_INF("fcb fap:");
    LOG_INF("fcb fap.fa_id = 0x%x", datalog_fcb.fap->fa_id);
    LOG_INF("fcb fap.fa_device_id = 0x%x", datalog_fcb.fap->fa_device_id);
    LOG_INF("fcb fap.fa_off = 0x%x", datalog_fcb.fap->fa_off);
    LOG_INF("fcb fap.fa_size = 0x%x", datalog_fcb.fap->fa_size);
    LOG_INF("fcb fap.fa_dev_name = %s", datalog_fcb.fap->fa_dev_name);
    LOG_INF("");

    uint32_t numEntries = DataLogGetNumEntries();
    uint32_t numBytes = DataLogGetLogSize();
    LOG_INF("Num Log entries %d, size %d", numEntries, numBytes);

    if (k_timer_remaining_get(&dlog_log_interval_timer) == 0)
    {
        uint16_t threshold = HAS_DATA_THRESHOLD_DEF;

        if (numEntries >= threshold)
        {
            LOG_INF("Log timer exp-HAS_DATA=1 (high water)");
        }
        else
        {
            LOG_INF("Log timer exp-HAS_DATA=0");
        }
    }
    else
    {
        LOG_INF("LogIntvl timer running-HAS_DATA force 0");
        clear_ble_ad_flag_bit(ADV_FLAGS_HAS_LOG_DATA);
    }

    return 0;
}

static int32_t cmd_log_scan(const struct shell *shell, size_t argc, char **argv)
{
    char *p;
    uint8_t flags = 0;
    if (argc > 1)
    {
        flags = (uint32_t)strtoul(argv[1], &p, 0);
    }
    cliScanLogForFlags_work.params.flags = flags;
    k_work_submit(&cliScanLogForFlags_work.work);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dlog,
                               SHELL_CMD(erase, NULL, "erase entire data log memory", cmd_log_erase),
                               SHELL_CMD(add, NULL, "Add test dataset.", cmd_log_add),
                               SHELL_CMD(read, NULL, "Read data log record.  (<index> <EntryOrRaw> <RawLen>).", cmd_log_read),
                               SHELL_CMD(get, NULL, "Get next unacked data log record. (<num>).", cmd_log_getnext),
                               SHELL_CMD(complete, NULL, "Complete read. (<WasRead>).", cmd_log_complete),
                               SHELL_CMD(ack, NULL, "Ack all pending entries.", cmd_log_ack),
                               SHELL_CMD(info, NULL, "Get data log info.", cmd_log_info),
                               SHELL_CMD(scan, NULL, "Scan data log for specified flags.  (<flag>)", cmd_log_scan),

                               SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(dlog, &sub_dlog, "DataLog commands", NULL);

//=================================================================================================
// end
//=================================================================================================
