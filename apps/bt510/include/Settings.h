//=================================================================================================
//! @file Settings.h
//!
//! @brief This is the header file for Settings.c.  It contains global
//!  function prototypes and application settings definitions.
//!
//! @copyright � 2020 Laird Connectivity, Inc
//!            Not for further distribution. All rights reserved.
//=================================================================================================

#ifndef SETTINGS_H
#define SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

//=================================================================================================
// Includes
//=================================================================================================


//=================================================================================================
// Global Constants, Macros and Type Definitions
//=================================================================================================
typedef enum
{
    ID_ACCEL_SAMPLING_INTERVAL, //0
    ID_NETWORK_ID,

    SETTINGS_COUNT,//2
} eSettingsId_t;

// When adding or deleting settings make sure the SETTING_SIZE defines are
// maintained.  These are used by the CLI to display settings.
#define SETTING_ARRAY_MAX (sizeof(uint32_t))
#define SETTING_SIZE_MAX (sizeof(uint32_t))
#define SETTING_PRINTABLE_ARRAY_MAX 72

struct Settings_s
{
    uint32_t accelSampIntervalSec;
    uint16_t networkId;

} __attribute__((packed));

typedef struct Settings_s Settings_t;

typedef enum
{
    UINT32 = 0,
    UINT16,
    UINT8,
    INT32,
    INT16,
    INT8,

    TYPE_LAST,

} eSettingType_t;

typedef enum
{
    SETTINGS_PERMISSION_NO_ACCESS,
    SETTINGS_PERMISSION_WRITE_ONLY,
    SETTINGS_PERMISSION_READ_ONLY,
    SETTINGS_PERMISSION_READ_WRITE,
} eSettingPerm_t;

struct SettingsDescription_s
{
  eSettingsId_t   id;
  eSettingType_t  type;
  eSettingPerm_t  permission;
  uint32_t        size;
  uint32_t        offset;
  uint8_t       * pName;
  void          * pDefault;

} __attribute__((packed));

typedef struct SettingsDescription_s SettingsDescription_t;

struct eSettingTypeInfo_s
{
  uint8_t         len;
  uint8_t       * pTypeStr;
} __attribute__((packed));

typedef struct eSettingTypeInfo_s eSettingTypeInfo_t;

//=================================================================================================
// Global Data Definitions
//=================================================================================================


//=================================================================================================
// Global Function Prototypes
//=================================================================================================
/**
 * @brief  Init the NV settings module.
 *
 */
void SettingsInit();

/**
 * @brief  Read the settings from the file in the filesystem and save to RAM copy
 *
 */
void SettingsReadFromFile();

/**
 * @brief  Get the value of a user non-volatile setting.
 *
 * @param  id - The ID of the setting to get.
 * @param  pValue - Pointer to storage where the read value will be stored.
 * @param  len - Number of bytes to be copied.
 *
 * @return true on success, or false on failure.
 */
bool SettingGetValue(eSettingsId_t id, uint8_t *pValue, uint32_t len);

/**
 * @brief  Set the value of an integer setting.
 *
 * @param  id - The ID of the setting to set.
 * @param  pValue - Pointer to storage where the new value will be written from.
 * @param  len - Number of bytes to be copied.
 *
 * @return true on success, or false on failure.
 */
bool SettingSetValue(eSettingsId_t id, uint8_t* pValue, uint32_t len);

/**
 * @brief  Update device ID table to given MAC address if current setting is set to default (all FFs)
 *
 * @param  addr - pointer to new MAC
 */
void UpdateSettingsDeviceIdDefault(uint8_t *addr);

/**
 * @brief  Remove the settings file and write out a new one with default values
 *
 */
void ResetAllSettingsToDef(void);

#ifdef __cplusplus
}
#endif

#endif // #ifndef SETTINGS_H


//=================================================================================================
// end
//=================================================================================================
