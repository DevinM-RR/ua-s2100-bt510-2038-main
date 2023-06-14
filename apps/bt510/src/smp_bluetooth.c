/******************************************************************************
 * @file smp_bluetooth.c
 * 
 * @brief implements bluetooth transport for SMP protocol. Based on smp_svr
 * sample from zephyr repo.
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <mgmt/mcumgr/smp_bt.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_vs.h>
#include <sys/byteorder.h>
#include <stdio.h>
#include <shell/shell.h>

#include <settings/settings.h>

#include "smp.h"
#include "systime.h"
#include "battery.h"
#include "led.h"
#include "appversion.h"
#include "fs_mgmt_handlers.h"
#include "DataLog.h"
#include "Settings.h"

//=================================================================================================
// Local Constant, Macro and Type Definitions
//=================================================================================================

#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>
LOG_MODULE_REGISTER(smp_bt);

#define ADV_DATA_MFG_IDX 1 //index of mfg field in advertising data

//Data indices of fields within manufacturing data (common for the PROTOCOL_ID_TRACKER protocol)
#define ADV_DATA_MFG_PROTOCOL_ID 2
#define ADV_DATA_MFG_NWK_IDX 4
#define ADV_DATA_MFG_FLAG_IDX 6
#define ADV_DATA_MFG_ID_ROT_IDX 8
#define ADV_DATA_MFG_ADV_REC_TYPE_IDX 14

//Data indices of fields for Adv Record Types: ADV_REC_TYPE_V20
#define ADV_DATA_MFG_DEV_TYPE_IDX 15
#define ADV_DATA_MFG_EPOCH_TIME_IDX 16
#define ADV_DATA_MFG_TX_PWR_IDX 20
#define ADV_DATA_MFG_ACCEL_MOTION_IDX 21
#define ADV_DATA_MFG_HW_MODEL_ID_IDX 22
#define ADV_DATA_MFG_BATTERY_MV_IDX 23
#define ADV_DATA_MFG_MAJOR_MINOR_IDX 24
#define ADV_DATA_MFG_BUILD_IDX 25

#define UPDATE_CONN_PARAMS_DELAY_TICKS K_MSEC(100)

#define UPDATE_PARAM_INTERVAL_MIN 6
#define UPDATE_PARAM_INTERVAL_MAX 8
#define UPDATE_PARAM_LATENCY 0
#define UPDATE_PARAM_TIMEOUT 200

#define SCAN_INTERVAL_VAL_IF_0 60

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static struct bt_le_conn_param conn_update_params = {
    .interval_min = UPDATE_PARAM_INTERVAL_MIN,
    .interval_max = UPDATE_PARAM_INTERVAL_MAX,
    .latency = UPDATE_PARAM_LATENCY,
    .timeout = UPDATE_PARAM_TIMEOUT,
};

typedef struct __DEVICE_AD_T__
{
    bt_addr_le_t addr;
    int8_t rssi;
} device_ad_t;

typedef struct __TIME_SOURCE_T
{
    uint8_t timeSource;
    uint32_t lastSet;
} time_source_t;

//=================================================================================================
// Local Function Prototypes
//=================================================================================================
static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl);
static void read_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl);
static void find_nearest_tx_power(int8_t *txPwr);
static void advertise(struct k_work *work);
static void advertise_stop(struct k_work *work);
static void update_conn_params_work_callback(struct k_work *work);

static void update_advert(struct k_work *work);
static void update_advert_timer_handler(struct k_timer *dummy);

static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void bt_start(struct k_work *work);
static void bt_ready(int err);

//=================================================================================================
// Global Data Definitions
//=================================================================================================

//=================================================================================================
// Local Data Definitions
//=================================================================================================

static bool smp_bluetooth_started = false;
static struct bt_conn *activeConn = NULL; //only tracks one connection at a time
static uint16_t active_conn_handle;

static uint16_t nwk_id = NWK_ID_DEF;
static uint16_t adv_interval = ADV_INT_MS_DEF;
static uint8_t device_type = DEVICE_TYPE_DEF;
static int8_t txPwr = TX_POWER_DEF;

static struct k_work advertise_work;
static struct k_work advertise_stop_work;
static struct k_work bt_start_work;
static struct k_delayed_work update_conn_params_work;

static char ad_mfg_data[] =
    {
        0x77, 0x00,                         /* Manufacturer ID */
        (uint8_t)REGAL_SENSOR_AD_PROTOCOL_ID, (uint8_t)(REGAL_SENSOR_AD_PROTOCOL_ID >> 8),                         /* Protocol ID (LSB, MSB) */
        0xCF, 0xCF,                         /* Network ID (LSB, MSB) */
        0x00, 0x00,                         /* Flags (LSB, MSB) */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Rotating Device ID */
        ADV_REC_TYPE_V20,                   /* Adv Data Record Type */
        DEVICE_TYPE_DEF,                        /* Device Type */
        0x00, 0x00, 0x00, 0x00,             /* epoch time */
        0x00,                               /* tx power */
        0x00,                               /* motion magnitude */
        0x00,                               /* hw model id */
        0x00,                               /* battery (ADC counts) */
        (APP_VERSION_MAJOR << 4) | (APP_VERSION_MINOR), /* Major Version and Minor Version */
        APP_VERSION_BUILD,                  /* Build */
    };

//CT ad format
static const struct bt_data adData[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),                    //3
    BT_DATA(BT_DATA_MANUFACTURER_DATA, ad_mfg_data, sizeof(ad_mfg_data))}; //28 bytes (include tag and len)

static uint8_t deviceName[] = {'R', 'E', 'G', 'A', 'L', '-', 'X', 'X', 'X', 'X', '\0'};

static struct k_work update_advert_work;

K_TIMER_DEFINE(update_advert_timer, update_advert_timer_handler, NULL);

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static const int8_t radio_tx_values[] = {
    -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8};

static bool advertEnabled = true;

static time_source_t timeSrc;

//=================================================================================================
// Global Function Definitions
//=================================================================================================
void smp_bt_init(void)
{
    smp_bluetooth_started = false;
}

void start_smp_bluetooth(void)
{
    if (!smp_bluetooth_started)
    {
        k_work_init(&advertise_work, advertise);
        k_work_init(&advertise_stop_work, advertise_stop);
        k_work_init(&bt_start_work, bt_start);
        k_work_init(&update_advert_work, update_advert);
        k_delayed_work_init(&update_conn_params_work,
                            update_conn_params_work_callback);

        advertEnabled = true;
        /* start bluetooth from system work q thread */
        k_work_submit(&bt_start_work);
    }
}

void stop_smp_bluetooth(void)
{
    if (smp_bluetooth_started)
    {
        advertEnabled = false;

        k_work_submit(&advertise_stop_work);

        //disconnect (otherwise will stay connected in "sleep off" mode)
        if (activeConn != NULL)
        {
            bt_conn_disconnect(activeConn, BT_HCI_ERR_LOCALHOST_TERM_CONN);
            bt_conn_unref(activeConn);
            activeConn = NULL;
        }
    }
}

void disconnect_if_connected(void)
{
    if (activeConn != NULL)
    {
        bt_conn_disconnect(activeConn, BT_HCI_ERR_LOCALHOST_TERM_CONN);
        bt_conn_unref(activeConn);
        activeConn = NULL;
    }
}

void restart_smp_bluetooth(void)
{
    if (smp_bluetooth_started)
    {
        advertEnabled = true;
        k_work_submit(&advertise_work);
    }
}

void set_ble_ad_flag_bit(uint16_t flag)
{
    uint8_t *pAdvFlag = (uint8_t *)&adData[ADV_DATA_MFG_IDX].data[ADV_DATA_MFG_FLAG_IDX];
    *pAdvFlag |= flag;
    if (!smp_dfu_in_progress()) 
    {
        LOG_DBG("Adv Flag data changed to (%x)", *pAdvFlag);
    }
}

void clear_ble_ad_flag_bit(uint16_t flag)
{
    uint8_t *pAdvFlag = (uint8_t *)&adData[ADV_DATA_MFG_IDX].data[ADV_DATA_MFG_FLAG_IDX];
    *pAdvFlag &= ~flag;
    if (!smp_dfu_in_progress()) 
    {
        LOG_DBG("Adv Flag data changed to (%x)", *pAdvFlag);
    }
}

bool is_set_ble_ad_flag_bit(uint16_t flag)
{
    uint8_t *pAdvFlag = (uint8_t *)&adData[ADV_DATA_MFG_IDX].data[ADV_DATA_MFG_FLAG_IDX];
    return ((*pAdvFlag & flag) == flag);
}

bool get_bt_address(bt_addr_le_t *addrs)
{
    bool success = false;

    if (smp_bluetooth_started)
    {
        size_t cnt = CONFIG_BT_ID_MAX;
        bt_id_get(addrs, &cnt);
        success = true;
    }

    return success;
}

void set_time_src(uint8_t src)
{
    timeSrc.timeSource = src;
    timeSrc.lastSet = SysTimeGet().Seconds;

    LOG_DBG("Time src changed to %d (lastSet = %d)", timeSrc.timeSource, timeSrc.lastSet);
}

bool smp_bluetooth_is_started(void)
{
    return smp_bluetooth_started;
}

bool isBleConnected(void)
{
    if (activeConn != NULL)
    {
        return true;
    }

    return false;
}

uint16_t get_nwk_id()
{
    return nwk_id;
}

uint16_t get_adv_interval()
{
    return adv_interval;
}

uint8_t get_device_type()
{
    return device_type;
}

int8_t get_tx_power()
{
    return txPwr;
}

//=================================================================================================
// Local Function Definitions
//=================================================================================================

static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
{
    struct bt_hci_cp_vs_write_tx_power_level *cp;
    struct bt_hci_rp_vs_write_tx_power_level *rp;
    struct net_buf *buf, *rsp = NULL;
    int err;

    buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
                            sizeof(*cp));
    if (!buf)
    {
        LOG_ERR("Unable to allocate command buffer");
        return;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle);
    cp->handle_type = handle_type;
    cp->tx_power_level = tx_pwr_lvl;

    err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
                               buf, &rsp);
    if (err)
    {
        uint8_t reason = rsp ? ((struct bt_hci_rp_vs_write_tx_power_level *)
                                 rsp->data)
                                ->status
                          : 0;
        LOG_ERR("Set Tx power err: %d reason 0x%02x", err, reason);
        return;
    }

    rp = (void *)rsp->data;
    //printk("Actual Tx Power: %d\n", rp->selected_tx_power);

    net_buf_unref(rsp);
}

static void read_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl)
{
    struct bt_hci_cp_vs_read_tx_power_level *cp;
    struct bt_hci_rp_vs_read_tx_power_level *rp;
    struct net_buf *buf, *rsp = NULL;
    int err;

    *tx_pwr_lvl = 0xFF;
    buf = bt_hci_cmd_create(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
                            sizeof(*cp));
    if (!buf)
    {
        LOG_ERR("Unable to allocate command buffer");
        return;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle);
    cp->handle_type = handle_type;

    err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
                               buf, &rsp);
    if (err)
    {
        uint8_t reason = rsp ? ((struct bt_hci_rp_vs_read_tx_power_level *)
                                 rsp->data)
                                ->status
                          : 0;
        LOG_DBG("Read Tx power err: %d reason 0x%02x", err, reason);
        return;
    }

    rp = (void *)rsp->data;
    *tx_pwr_lvl = rp->tx_power_level;

    net_buf_unref(rsp);
}

static void find_nearest_tx_power(int8_t *pPwr)
{
    int i;
    for (i = 0; i < sizeof(radio_tx_values); i++)
    {
        if (radio_tx_values[i] >= *pPwr)
        {
            *pPwr = radio_tx_values[i];
            return;
        }
    }
    // default to +8 if not found
    *pPwr = 8;
}

int8_t set_tx_power_from_settings(void)
{
    int8_t pwr = txPwr;
    find_nearest_tx_power(&pwr);

    if (!smp_bluetooth_started)
    {
        return pwr;
    }

    //printk("Set Tx power level to %d\n", pwr);
    set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV,
                 0, pwr);

    //printk("Get Tx power level -> ");
    read_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV,
                 0, &pwr);
    //printk("TXP = %d\n", pwr);
    return pwr;
}

static void advertise(struct k_work *work)
{
    int rc;
    uint16_t adIntervalMin = adv_interval;
    uint16_t adIntervalMax = adIntervalMin + 80;

    adIntervalMin = (uint16_t)(adIntervalMin / 0.625f);
    adIntervalMax = adIntervalMin + 80; /* + 50ms */

    bt_le_adv_stop();

    rc = bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE |
                                             BT_LE_ADV_OPT_USE_NAME,
                                         adIntervalMin,
                                         adIntervalMax, NULL),
                         adData, ARRAY_SIZE(adData), NULL, 0);
    if (rc)
    {
        LOG_ERR("Advertising failed to start (rc %d)", rc);
        return;
    }

    txPwr = set_tx_power_from_settings();
    LOG_INF("Advertising started (txpwr: %s%d dBm, %d-%dms)", (txPwr > 0) ? "+" : " ", txPwr, (uint16_t)(adIntervalMin * 0.625f), (uint16_t)(adIntervalMax * 0.625f));

    //Update advert every second to keep epoch time up to date
    k_timer_start(&update_advert_timer, K_MSEC(1000), K_MSEC(1000));
}

static void advertise_stop(struct k_work *work)
{
    bt_le_adv_stop();

    LOG_INF("Advertising stopped");
}

static void update_conn_params_work_callback(struct k_work *work)
{
    ARG_UNUSED(work);
    int err;

    /* Update connection params to reduce connection interval */
    if (activeConn)
    {
        err = bt_conn_le_param_update(activeConn, &conn_update_params);
        if (err)
        {
            LOG_DBG("conn update failed (err %d).", err);
        }
        else
        {
            LOG_DBG("conn update initiated.");
        }
    }
}

static void update_advert(struct k_work *work)
{
    uint8_t *pAdv = NULL;

    //Update network
    uint16_t localNwkId = NWK_ID_DEF;
    SettingGetValue(ID_NETWORK_ID, (uint8_t *)&localNwkId, sizeof(localNwkId));
    pAdv = (uint8_t *)&adData[ADV_DATA_MFG_IDX].data[ADV_DATA_MFG_NWK_IDX];
    memcpy(pAdv, (uint8_t *)&localNwkId, sizeof(localNwkId));

    // Update SMP runtime stats, including sampling battery voltage
    smp_update_uptime();

    // Update battery voltage in advert data
    uint32_t battmv = ADC_GetLastBatteryMv();
    uint8_t *pBatt = (uint8_t *)&adData[ADV_DATA_MFG_IDX].data[ADV_DATA_MFG_BATTERY_MV_IDX];
    *pBatt = (uint8_t)((battmv >> 4) & 0xFF);

    //Update low battery flag directly in ad array to take immediate effect
    bool battery_low = ADC_IsBatteryLow();
    uint8_t *pAdvFlags = (uint8_t *)&adData[ADV_DATA_MFG_IDX].data[ADV_DATA_MFG_FLAG_IDX];
    if (battery_low)
    {
        *pAdvFlags = (*pAdvFlags | ADV_FLAGS_LOW_BATTERY); // set the low battery flag bit
    }
    else
    {
        *pAdvFlags = (*pAdvFlags & ~ADV_FLAGS_LOW_BATTERY); // clear the low battery flag bit
    }

    //The following fields are defined for Advert Record Types: ADV_REC_TYPE_V20

    //Update device type
    uint8_t type = 0;
    type = get_device_type();
    pAdv = (uint8_t *)&adData[ADV_DATA_MFG_IDX].data[ADV_DATA_MFG_DEV_TYPE_IDX];
    memcpy(pAdv, (uint8_t *)&type, sizeof(type));

    //Update epoch time
    uint8_t flags = adData[ADV_DATA_MFG_IDX].data[ADV_DATA_MFG_FLAG_IDX];
    SysTime_t currTime;
    currTime.Seconds = 0; //init to 0
    if (flags & ADV_FLAGS_HAS_EPOCH_TIME)
    {
        currTime = SysTimeGet();
    }
    pAdv = (uint8_t *)&adData[ADV_DATA_MFG_IDX].data[ADV_DATA_MFG_EPOCH_TIME_IDX];
    memcpy(pAdv, (uint8_t *)&currTime.Seconds, sizeof(currTime.Seconds));

    //Update tx power
    int8_t txPwr = TX_POWER_DEF;
    get_tx_power();
    pAdv = (uint8_t *)&adData[ADV_DATA_MFG_IDX].data[ADV_DATA_MFG_TX_PWR_IDX];
    memcpy(pAdv, (uint8_t *)&txPwr, sizeof(txPwr));

    int32_t rc = bt_le_adv_update_data(adData, ARRAY_SIZE(adData), NULL, 0);
    if (rc && rc != -EAGAIN)
    {
        //Only print error if it is not -EAGAIN which just indicates that advertising hasn't started yet
        LOG_ERR("Adv data update failure (rc %d)", rc);
        return;
    }
}

/*
 * Add work to system work queue to update advertisement
 */
static void update_advert_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&update_advert_work);
}

static void bt_start(struct k_work *work)
{
    /* Enable Bluetooth. */
    int rc = bt_enable(bt_ready);

    if (rc != 0)
    {
        LOG_ERR("Bluetooth init failed (err %d)", rc);
        return;
    }
    else
    {
        LOG_DBG("Bluetooth init success");
    }

    bt_conn_cb_register(&conn_callbacks);

    /* Initialize the Bluetooth mcumgr transport. */
    smp_bt_register();
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err)
    {
        LOG_ERR("Connection failed (err 0x%02x)", err);
    }
    else
    {
        LOG_INF("Connected");

        activeConn = bt_conn_ref(conn);
        int32_t ret = bt_hci_get_conn_handle(activeConn,
                                           &active_conn_handle);
        if (ret)
        {
            LOG_ERR("No connection handle (err %d)\n", ret);
        }

        //stop advertising
        k_work_submit(&advertise_stop_work);

        /* Not sure if explicitly sending a connection parameter
         * update is necessary as it doesn't seem to make
         * a difference in performance in early testing.
         * Consider removing if not necessary.
         */
        // k_delayed_work_submit(&update_conn_params_work,
        //                       UPDATE_CONN_PARAMS_DELAY_TICKS);

        FsMgmtConnectedCb();
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    k_delayed_work_cancel(&update_conn_params_work); /* cancel connection parameter update if pending */
    LOG_INF("Disconnected (reason 0x%02x)", reason);
    if (activeConn != NULL)
    {
        bt_conn_unref(activeConn);
        activeConn = NULL;
    }

    /* stop blinking the LEDs if they were blinking due to DFU */
    if (led_get_pattern() == LED_PATTERN_DFU_IN_PROGRESS)
    {
        led_trigger_pattern(LED_PATTERN_OFF);
    }

    //Rather than stopping the scan interval timer, may be better to
    //keep scan interval timer running but not start scan (while in connection) so that scan start is more periodic.

    //Don't restart if stop bt has been called, so check enable flags here
    if (advertEnabled == true)
    {
        restart_smp_bluetooth(); //restart advertising and scanning
    }

    /* if DFU was in progress, turn off LEDs */
    smp_dfu_disconnected();
    
    DataLogDisconnectDb();
    FsMgmtDisconnectedCb();
}

static void bt_ready(int err)
{
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");
    smp_bluetooth_started = true;

    //Set BT name
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t cnt = CONFIG_BT_ID_MAX;
    bt_id_get(addrs, &cnt);

    sprintf(&deviceName[6], "%02X%02X", addrs[0].a.val[1], addrs[0].a.val[0]);
    bt_set_name(deviceName);

    if (advertEnabled == true)
    {
        /* Update the ad from the adFlag for the first time, then start advertising */
        k_work_submit(&advertise_work);
    }
}

static int cmd_bt_enable(const struct shell *shell, size_t argc, char **argv)
{
    if (!smp_bluetooth_started)
    {
        LOG_INF("BT not initialized!");
        return 0;
    }

    if (argc > 1)
    {
        if (strcmp(argv[1], "ad") == 0)
        {
            advertEnabled = true;
            k_work_submit(&advertise_work);
        }
        else
        {
            LOG_INF("Unknown command");
        }
    }

    return 0;
}

static int cmd_bt_disable(const struct shell *shell, size_t argc, char **argv)
{
    if (!smp_bluetooth_started)
    {
        LOG_INF("BT not initialized!");
        return 0;
    }

    if (argc > 1)
    {
        if (strcmp(argv[1], "ad") == 0)
        {
            advertEnabled = false;
            k_work_submit(&advertise_stop_work);
        }
        else if (strcmp(argv[1], "all") == 0)
        {
            advertEnabled = false;
            k_work_submit(&advertise_stop_work);
        }
        else
        {
            LOG_INF("Unknown command");
        }
    }

    return 0;
}

static int cmd_bt_gettxpwr(const struct shell *shell, size_t argc, char **argv)
{
    int8_t txPwr;

    read_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV,
                 0, &txPwr);

    LOG_INF("Adv pwr: %d", txPwr);

    read_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_SCAN,
                 0, &txPwr);

    LOG_INF("Scan pwr: %d", txPwr);

    read_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
                 0, &txPwr);

    LOG_INF("Conn pwr: %d", txPwr);

    return 0;
}

static int cmd_bt_addr(const struct shell *shell, size_t argc, char **argv)
{
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t cnt = CONFIG_BT_ID_MAX;
    bt_id_get(addrs, &cnt);

    LOG_INF("Address (%d): %04x%04x%04x", cnt, *(uint16_t*)&addrs[0].a.val[4], 
    *(uint16_t*)&addrs[0].a.val[2], *(uint16_t*)&addrs[0].a.val[0]);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_btc,
                               SHELL_CMD(enable, NULL, "Enable advertising and/or scanning.", cmd_bt_enable),
                               SHELL_CMD(disable, NULL, "Disable advertising and/or scanning.", cmd_bt_disable),
                               SHELL_CMD(gettx, NULL, "Print TX power", cmd_bt_gettxpwr),
                               SHELL_CMD(addr, NULL, "Print BLE address", cmd_bt_addr),
                               SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(btc, &sub_btc, "BT commands", NULL);
