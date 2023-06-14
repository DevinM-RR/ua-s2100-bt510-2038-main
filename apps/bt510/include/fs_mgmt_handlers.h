/******************************************************************************
 * @file fs_mgmt_handlers.h
 * @brief custom mcumgr filesystem management handlers
 * 
 * see .c file for details
 * 
 * Copyright (c) 2020, Laird Connectivity
 *****************************************************************************/

#ifndef __FS_MGMT_HANDLERS__
#define __FS_MGMT_HANDLERS__

#include "mgmt/mgmt.h"
extern struct mgmt_group fs_mgmt_group;

void FsMgmtConnectedCb(void);
void FsMgmtDisconnectedCb(void);

#endif
