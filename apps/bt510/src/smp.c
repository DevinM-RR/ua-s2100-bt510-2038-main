/******************************************************************************
 * @file smp.c
 * 
 * @brief implements SMP protocol. Based on smp_svr sample from zephyr repo.
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/
#include <zephyr.h>
#include <stats/stats.h>

#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
#include <device.h>
#include <fs/fs.h>
#include "fs_mgmt/fs_mgmt.h"
#include <fs/littlefs.h>
#endif
#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
#include "os_mgmt/os_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
#include "img_mgmt/img_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_STAT_MGMT
#include "stat_mgmt/stat_mgmt.h"
#endif

#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>
LOG_MODULE_REGISTER(smp);

#include <dfu/mcuboot.h>

#include "storage.h"
#include "smp.h"
#include "led.h"
#include "fs_mgmt_handlers.h"

/* Define an SMP stats group
 * uptime: approximates seconds since boot
 * battmv: reports battery level in mV
 * tempC: reports temperature in degress C
 */
STATS_SECT_START(smp_svr_stats)
STATS_SECT_ENTRY(uptime)
STATS_SECT_ENTRY(battmv)
STATS_SECT_ENTRY(tempC)
STATS_SECT_END;

/* Assign a name to the `uptime` stat. */
STATS_NAME_START(smp_svr_stats)
STATS_NAME(smp_svr_stats, uptime)
STATS_NAME(smp_svr_stats, battmv)
STATS_NAME(smp_svr_stats, tempC)
STATS_NAME_END(smp_svr_stats);

/* Define an instance of the stats group. */
STATS_SECT_DECL(smp_svr_stats) smp_svr_stats;

static bool smp_bt_is_registered = false;
static bool dfu_did_start = false;

struct mcuboot_img_header img_header;
static int swap_type = BOOT_SWAP_TYPE_INVALID;

extern void ProtectJtag();//global function defined in main.c

void dfu_started(void);
void dfu_stopped(void);
void dfu_pending(void);
void dfu_confirmed(void);

img_mgmt_dfu_callbacks_t img_mgmt_callbacks = {
    .dfu_started_cb = dfu_started, 
    .dfu_stopped_cb = dfu_stopped,
    .dfu_pending_cb = dfu_pending,
    .dfu_confirmed_cb = dfu_confirmed
};


/*
 * Start Bluetooth advertising of SMP service if not already started
 */
void smp_start(void)
{
	if(!smp_bt_is_registered) {
#ifdef CONFIG_MCUMGR_SMP_BT
        start_smp_bluetooth();
#endif
		smp_bt_is_registered = true;
	}
	else
	{
		// already registered, just start advertising
		restart_smp_bluetooth();
	}
	
}

void smp_stop(void)
{
	stop_smp_bluetooth();
}

static void mgmt_event_callback(uint8_t opcode, uint16_t group, uint8_t id, void *arg)
{
	// called on management events
	//printk("op %d, group %d, id %d\n", opcode, group, id);
	if(opcode == 3 && group == 8 && id == 0 && (((struct mgmt_evt_op_cmd_status_arg *)arg)->status == 0)) {
		// file download chunk completed
	}
}

int smp_init(void) {
	int rc = STATS_INIT_AND_REG(smp_svr_stats, STATS_SIZE_32,
				    "rt_stats");

	if (rc < 0) {
		LOG_ERR("Error initializing stats system [%d]", rc);
	}

	/* Register the custom mcumgr command handlers. */
#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
	mgmt_register_group(&fs_mgmt_group);
#endif

	/* Register the built-in mcumgr command handlers. */
#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
	os_mgmt_register_group();
#endif
#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
	img_mgmt_register_group();
	img_mgmt_register_callbacks(&img_mgmt_callbacks);
#endif
#ifdef CONFIG_MCUMGR_CMD_STAT_MGMT
	stat_mgmt_register_group();
#endif

	mgmt_register_evt_cb(mgmt_event_callback);

	/* read in image header into static img_header for later fw version checking */
	smp_read_image_header(&img_header);

    smp_bt_init();

    return swap_type;
}

void smp_read_image_header(struct mcuboot_img_header* pImgHeader)
{
	int rc;

	/* If we just rebooted with a new image being tested, start up SMP right
	 * away to allow the remote device to confirm the image
	 */
	rc = boot_read_bank_header(FLASH_AREA_ID(image_0), pImgHeader, sizeof(struct mcuboot_img_header));
	if (rc < 0) {
		LOG_ERR("Error reading image_0 header [%d]", rc);
		return;
	}

	bool image_confirmed = boot_is_img_confirmed();
	LOG_DBG("boot_img_confirmed: %d", image_confirmed);
	
	swap_type = mcuboot_swap_type();
	LOG_DBG("    boot_swap_type: %d", swap_type);
}

void smp_update_uptime(void) {
	uint64_t uptime_ms = k_uptime_get();
	STATS_SET(smp_svr_stats, uptime, (uint32_t)((uptime_ms/1000) & 0xFFFFFFFF));
}

void smp_set_battmv(uint32_t batt_level_mv) {
	STATS_SET(smp_svr_stats, battmv, batt_level_mv);
}

void smp_set_temp(int32_t temp) {
	STATS_SET(smp_svr_stats, tempC, temp);
}

void dfu_started(void)
{
	// DFU upload started
	LOG_INF("<<DFU STARTED>>");

	dfu_did_start = true;
}

void dfu_stopped(void)
{
	// DFU upload stopped
	LOG_INF("<<DFU STOPPED>>");
	dfu_did_start = false;
}

void dfu_pending(void)
{
	// DFU test pending
	LOG_INF("<<DFU PENDING>>");
}

void dfu_confirmed(void)
{
	// DFU image confirmed
	LOG_INF("<<DFU CONFIRMED>>");
    k_sleep(K_MSEC(1000));
}

void smp_dfu_disconnected(void)
{
	if(dfu_did_start) {
		LOG_DBG("disconnect during DFU... awaiting re-connect");
	}
}

bool smp_dfu_in_progress(void)
{
	return dfu_did_start;
}
