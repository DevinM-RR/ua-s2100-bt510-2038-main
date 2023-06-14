//=================================================================================================
//!
#define THIS_FILE "Settings.c"
//!
//! @copyright � 2020 Laird Connectivity, Inc
//!            Not for further distribution. All rights reserved.
//=================================================================================================

/**
* \file Settings.c
*
* \brief This file contains all non-volatile settings for this application.
*/

//=================================================================================================
// Includes
//=================================================================================================
#include <zephyr.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <bluetooth/bluetooth.h>

#include "Settings.h"
#include "smp.h"
#include "storage.h"
#include "DataLog.h"
#include "battery.h"
#include "smp.h"

#include <device.h>
#include <fs/fs.h>
#include "fs_mgmt/fs_mgmt.h"
#include <fs/littlefs.h>

#include <shell/shell.h>
#include <shell/shell_uart.h>

#include <logging/log.h>

//=================================================================================================
// Local Constant, Macro and Type Definitions
//=================================================================================================
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(settings);

//=================================================================================================
// Global Data Definitions
//=================================================================================================

//=================================================================================================
// Local Data Definitions
//=================================================================================================

// Real system settings
static Settings_t Nv_Settings;

static struct k_work printNvSettings_work;
static struct k_work readSettingsFromFile_work;

// Default NV settings and settings info structure
//
// If new settings are added place them in the same location as in the
// eSettingsId_t structure and in the Settings_t structure and adjust the
// SETTINGS_COUNT definition accordingly.
//
// Note: The index values are used as the print order of the cli command
// "nv r all" (print all nv settings command).
static const SettingsDescription_t SettingsDescription[SETTINGS_COUNT] = {
    // settings
    //  {
    //    .id             = ,
    //    .type           = ,
    //    .permission     = ,
    //    .size           = ,
    //    .offset         = ,
    //    .pName          = ,
    //    .pDefault       = ,
    //  },
    /******************************************************************************/
    {
        .id = ID_ACCEL_SAMPLING_INTERVAL,
        .type = UINT32,
        .permission = SETTINGS_PERMISSION_READ_WRITE,
        .size = sizeof(Nv_Settings.accelSampIntervalSec),
        .offset = offsetof(Settings_t, accelSampIntervalSec),
        .pName = "accelSampIntervalSec",
        .pDefault = (uint32_t[]){DEFAULT_ACCEL_SAMPLING_INTERVAL},
    },
    {
        .id = ID_NETWORK_ID,
        .type = UINT16,
        .permission = SETTINGS_PERMISSION_READ_WRITE,
        .size = sizeof(Nv_Settings.networkId),
        .offset = offsetof(Settings_t, networkId),
        .pName = "networkId",
        .pDefault = (uint16_t[]){NWK_ID_DEF},
    },
};

const eSettingTypeInfo_t SettingTypeInfo[TYPE_LAST] =
    {
        [UINT32] =
            {
                .len = sizeof(uint32_t),
                .pTypeStr = "uint32_t",
            },
        [UINT16] =
            {
                .len = sizeof(uint16_t),
                .pTypeStr = "uint16_t",
            },
        [UINT8] =
            {
                .len = sizeof(uint8_t),
                .pTypeStr = "uint8_t",
            },
        [INT32] =
            {
                .len = sizeof(int32_t),
                .pTypeStr = "int32_t",
            },
        [INT16] =
            {
                .len = sizeof(int16_t),
                .pTypeStr = "int16_t",
            },
        [INT8] =
            {
                .len = sizeof(int8_t),
                .pTypeStr = "int8_t",
            },
};

//=================================================================================================
// Local Function Prototypes
//=================================================================================================
static void printNvSettings_work_handler(struct k_work *work);
static void ReadSettingsFromFile_work_handler(struct k_work *work);
static void ReadSettingsFromFile(bool applySettings);
static bool GetIndexFromId(eSettingsId_t id, uint16_t *pIndex);
static uint8_t *GetRamSettingPointer(uint16_t index);
static uint16_t GetIndexFromSettingName(char *name);
static bool SettingRangeCheck(eSettingsId_t id, uint8_t *pValue);
static void ApplyNewSetting(uint16_t id);

static void CopyDefSettingsToShadowRam(void);
static bool SaveParamsFile(struct fs_file_t *file);
static void PrintSettingValueHex(uint8_t *pBuf, uint8_t *pVal, uint8_t size, uint32_t count, uint32_t bufSize);
static void PrintNvSettingsAll(void);

//=================================================================================================
// Global Function Definitions
//=================================================================================================
void SettingsInit()
{
    LOG_DBG("Init settings");

    CopyDefSettingsToShadowRam();

    //todo - testing
    // char fname[] = SETTINGS_FILENAME;
    // LOG_DBG("***ERASE PARAMS.TXT***");
    // fs_unlink(fname);
    // struct fs_file_t file2;
    // int ret = fs_open(&file2, fname, FS_O_RDWR | FS_O_CREATE); //open existing or create new
    // if (ret < 0)
    //     LOG_DBG("test FAIL: open %s: %d", fname, ret);

    // SaveParamsFile(&file2);
    // fs_seek(&file2, 0, FS_SEEK_SET);
    // ret = fs_close(&file2);
    // if (ret < 0)
    //     LOG_DBG("test FAIL: close %s: %d", fname, ret);
    ///////

    ReadSettingsFromFile(false); //only load settings but don't apply yet as other modules have not been initialized
    k_work_init(&printNvSettings_work, printNvSettings_work_handler);
    k_work_init(&readSettingsFromFile_work, ReadSettingsFromFile_work_handler);
}

void ResetAllSettingsToDef(void)
{
    CopyDefSettingsToShadowRam();

    LOG_DBG("***ERASING %s***", SETTINGS_FILENAME);

    //Save defaults to file
    bool fileOpen = false;
    struct fs_file_t file;
    char fname[] = SETTINGS_FILENAME;
    int32_t ret = 0;
    fs_unlink(fname);

    ret = fs_open(&file, fname, FS_O_RDWR | FS_O_CREATE); //open existing or create new
    if (ret < 0)
    {
        LOG_DBG("%s open failed: %d", log_strdup(fname), ret);
        fileOpen = false;
    }
    else
    {
        LOG_DBG("%s opened successfully", log_strdup(fname));
        fileOpen = true;
    }

    if (fileOpen)
    {
        if (!SaveParamsFile(&file))
        {
            LOG_DBG("Wrote defaults successfully\n");
        }
        fs_close(&file);
    }
}

void SettingsReadFromFile()
{
    k_work_submit(&readSettingsFromFile_work);
}

bool SettingGetValue(eSettingsId_t id, uint8_t *pValue, uint32_t len)
{
    bool success = true;
    uint16_t index = 0;
    uint32_t size = 0;
    uint8_t *pSrc = NULL;
    uint8_t *pDst = NULL;

    success = GetIndexFromId(id, &index);

    if (!success)
    {
        return success;
    }

    size = SettingsDescription[index].size;

    if (len == size)
    {
        pDst = pValue;
        pSrc = GetRamSettingPointer(index);

        memcpy(pDst, pSrc, size);

        success = true;
    }
    else
    {
        LOG_DBG("SettingGetValue failed: Param length mismatch (id = 0x%x, %d != %d)!", id, len, size);
        success = false;
    }

    return (success);
}

uint32_t SettingGetValuePointer(eSettingsId_t id, uint8_t **ppValue)
{
    bool success;
    uint16_t index;

    success = GetIndexFromId(id, &index);

    if (!success)
    {
        return success;
    }

    *ppValue = GetRamSettingPointer(index);

    success = true;

    return (success);
}

uint32_t SettingGetDefaultValue(eSettingsId_t id, uint8_t *pValue, uint32_t len)
{
    bool success = true;
    uint16_t index = 0;
    uint32_t size = 0;
    uint8_t *pSrc = NULL;
    uint8_t *pDst = NULL;

    success = GetIndexFromId(id, &index);

    if (!success)
    {
        return false;
    }

    size = SettingsDescription[index].size;

    if (len == size)
    {
        pDst = pValue;
        pSrc = (uint8_t *)SettingsDescription[index].pDefault;

        memcpy(pDst, pSrc, size);

        success = true;
    }
    else
    {
        success = false;
    }

    return (success);
}

bool SettingSetValue(eSettingsId_t id, uint8_t *pValue, uint32_t len)
{
    bool success = true;
    uint16_t index = 0;
    uint32_t size = 0;
    uint8_t *pSrc = NULL;
    uint8_t *pDst = NULL;

    if (pValue == NULL)
    {
        LOG_DBG("NULL data pointer\n");
        return false;
    }

    success = GetIndexFromId(id, &index);

    if (!success)
    {
        return false;
    }

    size = SettingsDescription[index].size;

    if (len != size)
    {
        LOG_DBG("Incorrect data length (%d != %d)\n", len, size);
        success = false;
    }
    else //if (len == size)
    {
        pDst = GetRamSettingPointer(index);
        pSrc = pValue;

        /* If the new and old values are the same, do nothing */
        if (memcmp(pDst, pSrc, size) == 0)
        {
            success = true;
        }
        else
        {
            //Perform any necessary range checking
            success = SettingRangeCheck(id, pSrc);

            if (success) //range checking succeeded
            {
                memcpy(pDst, pSrc, size);
                success = true;

                //Save to file
                bool fileOpen = false;
                struct fs_file_t file;
                char fname[] = SETTINGS_FILENAME;
                int32_t ret = 0;

                ret = fs_open(&file, fname, FS_O_RDWR | FS_O_CREATE); //open existing or create new
                if (ret < 0)
                {
                    LOG_DBG("%s open failed: %d", log_strdup(fname), ret);
                    fileOpen = false;
                }
                else
                {
                    LOG_DBG("%s opened successfully", log_strdup(fname));
                    fileOpen = true;
                }

                if (fileOpen)
                {
                    SaveParamsFile(&file);
                    fs_close(&file);
                }

                ApplyNewSetting(id);
            }
            else
            {
                LOG_ERR("Range check failed on setting 0x%x", id);
            }
        }
    }
    return success;
}

//=================================================================================================
// Local Function Definitions
//=================================================================================================

static void ReadSettingsFromFile_work_handler(struct k_work *work)
{
    ReadSettingsFromFile(true);
}

static void ReadSettingsFromFile(bool applySettings)
{
    char fname[] = SETTINGS_FILENAME;
    struct fs_file_t file;
    int32_t ret = 0;
    struct fs_dirent dirent;

    ret = fs_stat(fname, &dirent);
    if (ret == 0)
    {
        LOG_INF("%s size %u bytes", log_strdup(fname), dirent.size);
    }
    else
    {
        LOG_DBG("%s not found %d", log_strdup(fname), dirent.size);
        dirent.size = 0;
    }

    ret = fs_open(&file, fname, FS_O_RDWR | FS_O_CREATE); //open existing or create new
    if (ret < 0)
    {
        LOG_DBG("%s open failed: %d", log_strdup(fname), ret);
    }
    else
    {
        LOG_DBG("%s opened successfully", log_strdup(fname));

        if (dirent.size > 0)
        {
            bool resaveFile = false;
            char str[1024]; //needs to be big enough to fit entire params.txt file
            int32_t numRead = fs_read(&file, str, sizeof(str));
            LOG_DBG("Read %d bytes", numRead);
            if (numRead > 0)
            {
                uint16_t readIdx = 0;
                bool abortParse = false;
                uint32_t numSettingsParsed = 0;

                while(readIdx < numRead)
                {
                    uint16_t id = SETTINGS_COUNT;
                    uint16_t index = SETTINGS_COUNT;
                    char *p;

                    id = strtoul(&str[readIdx], &p, 16);
                    if (*p != str[readIdx + 4])
                    {
                        abortParse = true;
                        break;
                    }
                    readIdx += 5;
                    // LOG_DBG("Reading id: %04X", id);

                    bool success = GetIndexFromId(id, &index);
                    if (success)
                    {
                        if(numSettingsParsed + 1 > SETTINGS_COUNT)
                        {
                            LOG_DBG("All known settings have been extracted");
                            break;
                        }
                        numSettingsParsed++;

                        uint16_t size = SettingsDescription[index].size;
                        uint32_t tempData;
                        uint8_t *pData = GetRamSettingPointer(index);
                        uint8_t pSettingDataTemp[SETTING_ARRAY_MAX]; //needs to be set to largest size setting

                        for (uint16_t i = 0; i < size; i++)
                        {
                            uint8_t byteData[3] = {0};
                            byteData[0] = str[readIdx++];
                            byteData[1] = str[readIdx++];
                            tempData = strtoul(byteData, &p, 16);

                            //Verify data is valid by ensuring that entire string was parsed
                            if (*p == byteData[2])
                            {
                                pSettingDataTemp[i] = (uint8_t)tempData;
                            }
                            else
                            {
                                LOG_ERR("Setting %x byte %d parsing failed (%c%c).", id, i, byteData[0], byteData[1]);
                                abortParse = true;
                                break;
                            }
                        }

                        bool success = SettingRangeCheck((eSettingsId_t)id, pSettingDataTemp);
                        if (success)
                        {
                            memcpy(pData, pSettingDataTemp, size);
                            if (applySettings)
                            {
                                ApplyNewSetting(id);
                            }
                        }
                        else
                        {
                            LOG_ERR("Range check failed on setting 0x%x. Keep prev val.", id);
                            resaveFile = true;
                        }

                        //After reading data, check for CR/NL
                        if (str[readIdx] == 0x0d || str[readIdx] == 0x0a)
                        {
                            readIdx++;
                        }
                        if (str[readIdx] == 0x0d || str[readIdx] == 0x0a)
                        {
                            readIdx++;
                        }
                    }
                    else
                    {
                        LOG_DBG("Could not find index for %d", id);
                        break; //can't continue as data size is not known for unknown ID
                    }

                    if (abortParse)
                    {
                        break;
                    }
                }

                LOG_DBG("Finshed parsing file %d out of %d (%d settings)", readIdx, numRead, numSettingsParsed);
                if(numSettingsParsed < SETTINGS_COUNT)
                {
                    //If the file did not contain all known settings, re-save file to make sure current settings are saved.
                    resaveFile = true;
                    LOG_DBG("Resave file to store all settings");
                }

                if (abortParse)
                {
                    //Parsing error. Reset to defaults, delete file and re-create from defaults.
                    LOG_ERR("File parsing failed. Reset to defaults.");
                    fs_close(&file); //Flushes buffers and saves to flash
                    fs_unlink(fname);
                    CopyDefSettingsToShadowRam();
                    fs_open(&file, fname, FS_O_RDWR | FS_O_CREATE); //create new
                    SaveParamsFile(&file);
                }
                else if (resaveFile)
                {
                    //save file out with new settings and any previous settings that failed range checking
                    fs_close(&file); //Flushes buffers and saves to flash
                    fs_unlink(fname);
                    fs_open(&file, fname, FS_O_RDWR | FS_O_CREATE); //create new
                    SaveParamsFile(&file);
                }
            }
            else
            {
                LOG_DBG("Read failed: %x", numRead);
            }
        }
        else
        {
            //Save defaults to file
            LOG_DBG("Blank params file (%d). Save defaults to file", dirent.size);
            SaveParamsFile(&file);
        }

        ret = fs_close(&file); //Flushes buffers and saves to flash
        if (ret < 0)
        {
            LOG_DBG("Params file close failed: %d", ret);
        }
        else
        {
            LOG_DBG("Params file closed successfully");
        }
    }
}

static bool GetIndexFromId(eSettingsId_t id, uint16_t *pIndex)
{
    bool success = false;
    uint16_t i;

    for (i = 0; i < SETTINGS_COUNT; i++)
    {
        if (SettingsDescription[i].id == id)
            break;
    }

    if (i < SETTINGS_COUNT)
    {
        *pIndex = i;
        success = true;
    }
    else
    {
        success = false;
    }

    return (success);
}

static uint8_t *GetRamSettingPointer(uint16_t index)
{
    uint8_t *pValue = (uint8_t *)&Nv_Settings;
    pValue += SettingsDescription[index].offset;

    return (pValue);
}

static void CopyDefSettingsToShadowRam(void)
{
    uint8_t *pSrc;
    uint8_t *pDst;
    uint32_t size;

    // This is a settings type record, load default value to all settings
    for (uint32_t setIdx = 0; setIdx < SETTINGS_COUNT; setIdx++)
    {
        pDst = GetRamSettingPointer(setIdx);
        pSrc = (uint8_t *)SettingsDescription[setIdx].pDefault;
        size = SettingsDescription[setIdx].size;
        memcpy(pDst, pSrc, size);
    }
}

static bool SaveParamsFile(struct fs_file_t *file)
{
    bool success = false;

    uint8_t *pSrc;
    uint32_t size;

    int32_t numSettings = SETTINGS_COUNT;
    for (uint32_t setIdx = 0; setIdx < numSettings; setIdx++)
    {
        char str[256] = {0x20};

        pSrc = GetRamSettingPointer(setIdx);
        size = SettingsDescription[setIdx].size;

        int32_t writeSize = 5; //4-char ID and equals sign
        sprintf(&str[0], "%04X=", SettingsDescription[setIdx].id);
        for (int32_t i = 0; i < size; i++)
        {
            sprintf(&str[writeSize], "%02X", pSrc[i]);
            writeSize += 2; //2-char byte
            if (writeSize > 256)
            {
                LOG_DBG("Data is too large  (>256)");
                break;
            }
        }
        sprintf(&str[writeSize], "\r\n");
        writeSize += 2;

        int32_t ret = fs_write(file, str, writeSize);
        if (ret > 0)
        {
            success = true;

            // LOG_DBG("Write success %d %d", ret, SettingsDescription[setIdx].id);
        }
        else
        {
            LOG_DBG("Write failed: %x", ret);
            success = false;
        }
    }

    return success;
}

static void PrintSettingValueHex(uint8_t *pBuf, uint8_t *pVal, uint8_t size, uint32_t count, uint32_t bufSize)
{
    if (size * count * 2 > bufSize)
    {
        LOG_DBG("Size exceeds buffer %d > %d!", size * count * 2, bufSize);
    }

    char *pDst = (char *)pBuf;
    uint8_t *pSrc = pVal;

    // Build the value string
    for (uint32_t i = 0; i < count; i++)
    {
        // If there are too many bytes in the array to display, truncate
        if ((i * (size * 2 + 1)) >= SETTING_PRINTABLE_ARRAY_MAX || (i * (size * 2 + 1)) >= (bufSize - 4))
        {
            snprintf(pDst, 4, "...");
            break;
        }

        if (size == 4)
        {
            snprintf(pDst, 10, "%08X ", *(uint32_t *)pSrc);
            pDst += 9; // 8 chars plus space
            pSrc += size;
        }
        else if (size == 2)
        {
            snprintf(pDst, 6, "%04X ", *(uint16_t *)pSrc);
            pDst += 5; // 4 chars plus space
            pSrc += size;
        }
        else
        {
            snprintf(pDst, 4, "%02X ", *(uint8_t *)pSrc);
            pDst += 3; // 2 chars plus space
            pSrc += 1;
        }
    }
}

static void PrintNvSettingsAll(void)
{
    uint16_t id;
    eSettingPerm_t perm;
    uint8_t *pVal;
    uint8_t *pNam;
    eSettingType_t valType;
    uint8_t *pType;
    uint32_t dataCount;
    uint8_t strBuf[SETTING_SIZE_MAX * 3 + SETTING_ARRAY_MAX + 1]; // Must be big enough to fit the largest setting size plus space plus NUL!
    uint8_t valBuf[SETTING_SIZE_MAX * 2];

    // Print a header
    printk("Settings Number: %u\r\n\n", SETTINGS_COUNT);
    printk("strBuf size: %u\r\n\n", sizeof(strBuf));
    printk("valBuf size: %u\r\n\n", sizeof(valBuf));

    printk("  ID            Name                    Type            Array   Value\r\n");
    printk("  ------------- ----------------------- --------------- ------- -------------------------------------------------\r\n");
    k_sleep(K_MSEC(100));
    for (uint16_t i = 0; i < SETTINGS_COUNT; i++)
    {
        id = SettingsDescription[i].id;
        pNam = SettingsDescription[i].pName;
        pVal = GetRamSettingPointer(i);
        valType = SettingsDescription[i].type;
        pType = SettingTypeInfo[valType].pTypeStr;
        perm = SettingsDescription[i].permission;

        // If permission for this setting is not readable, do not display it
        if (!(perm == SETTINGS_PERMISSION_READ_ONLY || perm == SETTINGS_PERMISSION_READ_WRITE))
            continue;

        // Get array size
        dataCount = SettingsDescription[i].size / SettingTypeInfo[valType].len;

        // Copy the setting to local storage and deal with endianess
        memset(strBuf, 0, sizeof(strBuf));
        memcpy(valBuf, pVal, SettingsDescription[i].size);

#if 0
    if(dataCount > 1)
    {
      ReverseHexDataArray(valBuf, SettingTypeInfo[valType].len, dataCount);
    }
#endif

        // Convert the setting to a printable string
        // Device ID setting seems to be too large and not all bytes are printed (including the <CR><NL>)
        PrintSettingValueHex(strBuf, valBuf, SettingTypeInfo[valType].len, dataCount, sizeof(strBuf));

        // Print the setting line
        printk("  0x%04X\t%-22s\t%-8s\t%u\t0x%s\r\n",
               id,
               pNam,
               pType,
               dataCount,
               strBuf);

        k_sleep(K_MSEC(dataCount * 20));
    }
}

/*
 * Print the NV settings using the system work q
 */
static void printNvSettings_work_handler(struct k_work *work)
{
    PrintNvSettingsAll();
}

/*
 * Get the index associated with the passed in param name
 * returns: index into the settings array or 0xFFFF if not found
 */
static uint16_t GetIndexFromSettingName(char *name)
{
    int i;
    for (i = 0; i < SETTINGS_COUNT; i++)
    {
        if (0 == strcmp(SettingsDescription[i].pName, name))
        {
            return i;
        }
    }
    LOG_INF("Unknown setting: %s", name);
    return 0xFFFF;
}

static bool SettingRangeCheck(eSettingsId_t id, uint8_t *pValue)
{
    bool success = true;

    if (id == ID_ACCEL_SAMPLING_INTERVAL)
    {
        //no range check
    }
    else if (id == ID_NETWORK_ID)
    {
        //no range check
    }

    return success;
}

static void ApplyNewSetting(uint16_t id)
{
    if (id == ID_ACCEL_SAMPLING_INTERVAL)
    {
        //should be handled by accel thread (short loop and check elapsed)
    }
    else if (id == ID_NETWORK_ID)
    {
        //advertisement will be updated automatically
    }
}

/*
 * Shell nv set command
 */
static int cmd_nv_set(const struct shell *shell, size_t argc, char **argv)
{
    char *p;
    uint32_t u32v;
    uint16_t u16v;
    uint8_t u8v;
    int32_t s32v;
    int16_t s16v;
    int8_t s8v;


if(k_is_in_isr())
{
    LOG_INF("is in isr");
}

    if (argc < 3)
    {
        LOG_WRN("must specify nv param name and value");
    }
    else
    {
        // Set the associatd nv settings
        int idx = GetIndexFromSettingName(argv[1]);
        if (idx < SETTINGS_COUNT)
        {
            bool success = true;
            switch (SettingsDescription[idx].type)
            {
            case UINT32:
                u32v = (uint32_t)strtoul(argv[2], &p, 10);
                success = SettingSetValue(idx, (uint8_t *)&u32v, sizeof(u32v));
                if (success)
                {
                    LOG_INF("set nv param %s to %u", log_strdup(argv[1]), u32v);
                }
                break;
            case UINT16:
                u16v = (uint16_t)strtoul(argv[2], &p, 10);
                success = SettingSetValue(idx, (uint8_t *)&u16v, sizeof(u16v));
                if (success)
                {
                    LOG_INF("set nv param %s to %u", log_strdup(argv[1]), u16v);
                }
                break;
            case UINT8:
                if (SettingsDescription[idx].size == sizeof(uint8_t))
                {
                    u8v = (uint8_t)strtoul(argv[2], &p, 10);
                    success = SettingSetValue(idx, (uint8_t *)&u8v, sizeof(u8v));
                    if (success)
                    {
                        LOG_INF("set nv param %s to %u", log_strdup(argv[1]), u8v);
                    }
                }
                else
                {
                    LOG_INF("arrays not implemented for shell access");
                }
                break;
            case INT32:
                s32v = (int32_t)strtol(argv[2], &p, 10);
                success = SettingSetValue(idx, (uint8_t *)&s32v, sizeof(s32v));
                if (success)
                {
                    LOG_INF("set nv param %s to %d", log_strdup(argv[1]), s32v);
                }
                break;
            case INT16:
                s16v = (int16_t)strtol(argv[2], &p, 10);
                success = SettingSetValue(idx, (uint8_t *)&s16v, sizeof(s16v));
                if (success)
                {
                    LOG_INF("set nv param %s to %d", log_strdup(argv[1]), s16v);
                }
                break;
            case INT8:
                s8v = (int8_t)strtol(argv[2], &p, 10);
                success = SettingSetValue(idx, (uint8_t *)&s8v, sizeof(s8v));
                if (success)
                {
                    LOG_INF("set nv param %s to %d", log_strdup(argv[1]), s8v);
                }
                break;
            default:
                LOG_INF("type '%s' not implemented for shell access", SettingTypeInfo[SettingsDescription[idx].type].pTypeStr);
                break;
            }
        }
    }

    return 0;
}

/*
 * Shell nv list command
 */
static int cmd_nv_list(const struct shell *shell, size_t argc, char **argv)
{
    k_work_submit(&printNvSettings_work);
    return 0;
}

static int cmd_nv_default(const struct shell *shell, size_t argc, char **argv)
{
    ResetAllSettingsToDef();
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_nv,
                               SHELL_CMD(set, NULL, "set a param.", cmd_nv_set),
                               SHELL_CMD(list, NULL, "list nv params.", cmd_nv_list),
                               SHELL_CMD(default, NULL, "set nv params to defaults.", cmd_nv_default),
                               SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(nv, &sub_nv, "NV commands", NULL);

//=================================================================================================
// end
//=================================================================================================
