/******************************************************************************
 * @file smp.h
 * @brief Simple management protocol service
 *
 * see .c file for details
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/

#ifndef __SMP_H__
#define __SMP_H__

#include <bluetooth/addr.h>
#include <dfu/mcuboot.h>

#define BT_MAC_ADDR_LEN                 sizeof(bt_addr_t)

#define ADV_FLAGS_HAS_EPOCH_TIME        0x0001
#define ADV_FLAGS_HAS_LOG_DATA          0x0002
#define ADV_FLAGS_HAS_MOTION            0x0004
#define ADV_FLAGS_LOW_BATTERY           0x0008
#define ADV_FLAGS_DATALOG_FULL          0x0010

#define ADV_REC_TYPE_V20                0x20

#define BOOT_SWAP_TYPE_INVALID          0xFF

#define TIME_RESYNC_PERIOD_SEC          3600//1 hour
#define TIME_RESYNC_MARGIN_DEF_SEC      60  //1 minute
#define TIME_RESYNCH_MARGIN_SEC_MINIMUM 5 //5 seconds

#define ADV_INT_MS_DEF 250
#define NWK_ID_DEF 0xCFCF //unused but needed for BLE transfer protocol
#define DEVICE_TYPE_DEF 0 //unused but needed for BLE transfer protocol
#define TX_POWER_DEF 8
#define REGAL_SENSOR_AD_PROTOCOL_ID      0xFF84
#define REGAL_GATEWAY_AD_PROTOCOL_ID     0xFF85

typedef enum
{
    TIME_SRC_NONE,
    TIME_SRC_SMP,
    TIME_SRC_BLE_CLI,
} eTimeSrc_t;

//MG100 can take around 20 seconds to process 40 one-record entries
//in a single packet and so timeout must be larger
#define SMP_TIMEOUT_TICKS               K_SECONDS(120)

#define STATS_SET(group__, var__, val__) \
	((group__).var__ = (val__))

int smp_init(void);
void smp_read_image_header(struct mcuboot_img_header *pImgHeader);
void smp_update_uptime(void);
void smp_set_battmv(uint32_t batt_level_mv);
void smp_set_temp(int32_t temp);
void smp_start(void);
void smp_stop(void);
void smp_dfu_disconnected(void);
bool smp_dfu_in_progress(void);

//smp_bluetooth functions;
void smp_bt_init(void);
void start_smp_bluetooth(void);
void stop_smp_bluetooth(void);
void restart_smp_bluetooth(void);
int8_t set_tx_power_from_settings(void);
void set_ble_ad_flag_bit(uint16_t flag);
void clear_ble_ad_flag_bit(uint16_t flag);
bool is_set_ble_ad_flag_bit(uint16_t flag);
bool get_bt_address(bt_addr_le_t *addrs);
extern struct mcuboot_img_header img_header;
void disconnect_if_connected(void);
void set_time_src(uint8_t src);
bool smp_bluetooth_is_started(void);
bool isBleConnected(void);
void bt_send_data(uint8_t *pData, uint8_t data);
uint16_t bt_total_devices_seen(void);
uint16_t get_nwk_id(void);
uint16_t get_adv_interval(void);
uint8_t get_device_type(void);
int8_t get_tx_power(void);

#endif
