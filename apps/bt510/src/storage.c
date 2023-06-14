/******************************************************************************
 * @file storage.c
 * 
 * @brief mounts a littlefs partition at '/lfs' from the flash 'storage'
 * partition.
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

#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(storage);

#include "storage.h"

#ifdef CONFIG_MCUMGR_CMD_FS_MGMT
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(cstorage);
static struct fs_mount_t littlefs_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &cstorage,
	.storage_dev = (void *)FLASH_AREA_ID(lfs_storage),
	.mnt_point = FS_MOUNT_STR
};
#endif

void storage_init(void) {

	/* mount littlefs at /lfs */
   	int rc = fs_mount(&littlefs_mnt);
	if (rc < 0) {
		LOG_ERR("Error mounting littlefs [%d]", rc);
	}

	/* init/load settings here */
	
}