/**********
 * custom fs_mgmt handlers
 * allows customization such as blinking of LED to incidate status
 */
#include <zephyr.h>
#include <stdlib.h>

#include "cborattr/cborattr.h"
#include "mgmt/mgmt.h"
#include "fs_mgmt/fs_mgmt.h"
#include "fs_mgmt/fs_mgmt_impl.h"
#include "fs_mgmt/fs_mgmt_config.h"

#include <drivers/flash.h>
#include <device.h>
#include <sys/crc.h>
#include <bluetooth/bluetooth.h> // just for definition of BT_ADDR_LE_STR_LEN
#include <crypto/cipher.h>

#include "led.h"
#include "sysTime.h"
#include "storage.h"
#include "Settings.h"
#include "accel.h"
#include "DataLog.h"
#include "smp.h"
#include "systime.h"
#include "battery.h"
#include "temperature.h"

#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(fs_mgmt);

static mgmt_handler_fn fs_mgmt_file_download;
static mgmt_handler_fn fs_mgmt_file_upload;

static struct {
    /** Whether an upload is currently in progress. */
    bool uploading;

    /** Expected offset of next upload request. */
    size_t off;

    /** Total length of file currently being uploaded. */
    size_t len;
} fs_mgmt_ctxt;

static const struct mgmt_handler fs_mgmt_handlers[] = {
    [FS_MGMT_ID_FILE] = {
        .mh_read = fs_mgmt_file_download,
        .mh_write = fs_mgmt_file_upload,
    },
};

#define FS_MGMT_HANDLER_CNT \
    (sizeof fs_mgmt_handlers / sizeof fs_mgmt_handlers[0])

struct mgmt_group fs_mgmt_group = {
    .mg_handlers = fs_mgmt_handlers,
    .mg_handlers_count = FS_MGMT_HANDLER_CNT,
    .mg_group_id = MGMT_GROUP_ID_FS,
};

/*
 * When DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL is available, we use it here.
 * Otherwise the device can be set at runtime with the set_device command.
 */
#ifndef DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL
#define DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL ""
#endif

#define FS_MGMT_MAX_PAYLOAD_SIZE CONFIG_MGMT_CBORATTR_MAX_SIZE
#define CBOR_OVERHEAD_SIZE 128
#define FS_MGMT_MAX_CBOR_BUF_SIZE (CONFIG_MGMT_CBORATTR_MAX_SIZE + CBOR_OVERHEAD_SIZE)

#define AES_IV_SIZE 16
#define MAX_CONSECUTIVE_LOG_READ_FAILURES 20

static uint16_t entry_size = 0;
static uint32_t max_entry_size = 0;
static uint32_t file_len;
static uint32_t last_upload_time = 0;

static uint32_t consecutiveLogReadFailures = 0;
static bool logReadCompleted = false;

static void populate_ct_header(struct ct_log_header *hdr)
{
    // get this device's BLE address
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t count = CONFIG_BT_ID_MAX;
    bt_id_get(addrs, &count);

    struct sensor_value temp;
    temp = TMP_GetLatestAmbient();

    hdr->entry_protocol_version = LOG_ENTRY_PROTOCOL_VERSION;
    hdr->entry_size = max_entry_size;
    hdr->entry_count = 0; // unused
    memcpy(hdr->device_id, addrs[0].a.val, sizeof(addrs[0].a.val));
    hdr->last_upload_time = last_upload_time;
    hdr->device_time = SysTimeGet().Seconds;
    last_upload_time = hdr->device_time;
    hdr->log_size = file_len;
    hdr->local_info.fw_version[0] = img_header.h.v1.sem_ver.major;
    hdr->local_info.fw_version[1] = img_header.h.v1.sem_ver.minor;
    *((uint16_t *)&hdr->local_info.fw_version[2]) = img_header.h.v1.sem_ver.revision;
    hdr->local_info.network_id = get_nwk_id();
    hdr->local_info.ad_interval_ms = get_adv_interval();
    hdr->local_info.temperature_int = temp.val1;
    hdr->local_info.temperature_frac = temp.val2;
    hdr->local_info.battery_level = (ADC_GetLastBatteryMv() >> 4) & 0xFF;
    hdr->local_info.deviceType = get_device_type();
    hdr->local_info.tx_power = get_tx_power();
    hdr->local_info.up_time_sec = (uint32_t)(k_uptime_get() / 1000);
}

static struct fcb_entry loc;
static bool entry_was_sent = false;

static uint8_t dl_file_data[FS_MGMT_MAX_CBOR_BUF_SIZE];
static uint8_t enc_dl_file_data[FS_MGMT_MAX_CBOR_BUF_SIZE];

#define DEBUG_CBOR_PAYLOADS 1

/**
 * Command handler: fs file (read)
 */
static int
fs_mgmt_file_download(struct mgmt_ctxt *ctxt)
{
    char path[FS_MGMT_PATH_SIZE + 1];
    unsigned long long off;
    CborError err;
    size_t bytes_read;
    int rc;
    int bytes_to_read;
    uint32_t current_chunk_len = 0; /* keep track of # of bytes copied into file_data buffer */
    bool end_transfer = false;
    uint16_t crcval;

    const struct cbor_attr_t dload_attr[] = {
        {
            .attribute = "off",
            .type = CborAttrUnsignedIntegerType,
            .addr.uinteger = &off,
        },
        {
            .attribute = "name",
            .type = CborAttrTextStringType,
            .addr.string = path,
            .len = sizeof path,
        },
        {0},
    };

    off = ULLONG_MAX;
    rc = cbor_read_object(&ctxt->it, dload_attr);
    if (rc != 0 || off == ULLONG_MAX) {
        return MGMT_ERR_EINVAL;
    }

    if(strncmp(path, "/log/ct", 7) == 0) {
       
#if DEBUG_CBOR_PAYLOADS
        printk("cbor off: %d\n", (uint32_t)off);
#endif
        if(off == 0) {
            // clear the log_available bit to avoid other gateways connecting to this device
            clear_ble_ad_flag_bit(ADV_FLAGS_HAS_LOG_DATA);

            /* Prep the data log for download */
            file_len = DataLogStartDownload();
            uint32_t ecnt = DataLogGetDownloadEntryCount();
#if DEBUG_CBOR_PAYLOADS
            printk("DataLogStartDownload(): %d, entry_count: %d, sizeof_ct_log_header (no crcs): %d\n", file_len, ecnt, sizeof(struct ct_log_header));
#endif

            if (file_len == 0)
            {
                //file_len of 0 means read will not be started.
                printk("Log size is 0 (empty or log is not ready)! Disconnect.\n");

                //DataLogWasRead is not called so that MG100 can re-connect to this device after scan completes

                return MGMT_ERR_EBADSTATE;
            }

            (void)memset(&loc, 0, sizeof(loc)); //set entry to 0 to get oldest entry
            entry_was_sent = false;

            // determine the length of the file here
            max_entry_size = LOG_ENTRY_SIZE;
            file_len = file_len + (ecnt * sizeof(uint16_t)); /* account for 16 bit CRC for each entry */
            uint32_t headerSize = (sizeof(struct ct_log_header) + sizeof(uint16_t));//header + header crc
            file_len = file_len + headerSize;

#if DEBUG_CBOR_PAYLOADS
            // Note: this print interferes with serial SMP file download
            printk("file_len (w/crcs): %d, entry_count: %d\n", file_len, ecnt);
#endif
        }
        else
        {
            // we've already sent the ct_log_header, now send an entry
            bytes_to_read = FS_MGMT_MAX_PAYLOAD_SIZE;//initialize to max - update after entries are read
            if(entry_was_sent) {
#if DEBUG_CBOR_PAYLOADS
                printk("calling DataLogAckEntries()\n");
#endif
                DataLogAckEntries(false);
            }
        }

        if (off == 0)
        {
            memset(enc_dl_file_data, 0, sizeof(enc_dl_file_data));
            memset(dl_file_data, 0, sizeof(dl_file_data));

            /* copy in the header */
            struct ct_log_header ct_hdr;
            populate_ct_header(&ct_hdr);
            memcpy(dl_file_data, &ct_hdr, sizeof(struct ct_log_header));
            crcval = crc16_ccitt(0, dl_file_data, sizeof(struct ct_log_header));
            *((uint16_t *)&dl_file_data[sizeof(struct ct_log_header)]) = crcval;

            /* first packet always just contains the ct_log_header */
            bytes_to_read = sizeof(struct ct_log_header) + sizeof(uint16_t);
        }
        else
        {
            memset(enc_dl_file_data, 0, sizeof(enc_dl_file_data));
            memset(dl_file_data, 0, sizeof(dl_file_data));

            current_chunk_len = 0;
            bytes_to_read = 0;
            uint32_t payloadBufferAvail = FS_MGMT_MAX_PAYLOAD_SIZE - sizeof(uint16_t);
            
            do {
                uint32_t availableLen = (FS_MGMT_MAX_PAYLOAD_SIZE/2) - sizeof(uint16_t) - current_chunk_len;//only use half of maximum payload to prevent MG100 issues
                
                rc = DataLogGetNextEntry(&loc, &dl_file_data[current_chunk_len], &entry_size, availableLen);
                if(rc == -ENOSPC)
                {
                    printk("entry > avail, sending (ccl: %d)\n", current_chunk_len);
                    break;
                }

#if DEBUG_CBOR_PAYLOADS
                printk("entry addr: %08X size: %d (%d/%d)\n", LOG_ADDR_START + FCB_ENTRY_FA_DATA_OFF(loc), entry_size, current_chunk_len, FS_MGMT_MAX_PAYLOAD_SIZE);
#endif

                if (entry_size > availableLen)
                {
                    /* entry size is too large for space available in this
                     * packet, just send what we have so far */
                    printk("entry > avail, sending  (ccl: %d)\n", current_chunk_len);
                    break;
                }
                else if (loc.fe_sector == 0 || entry_size == 0 || rc != 0)
                {
                    /* end the transfer, there are no more entries */
                    end_transfer = true;
#if DEBUG_CBOR_PAYLOADS
                    printk("end xfer, entry_size == %d\n", entry_size);
#endif
                    break;
                } else {
                    if ((current_chunk_len + entry_size + sizeof(uint16_t)) < FS_MGMT_MAX_PAYLOAD_SIZE)
                    {
                        /* calculate crc16 of the entry just copied into the buffer and append it */
                        crcval = crc16_ccitt(0, &dl_file_data[current_chunk_len], entry_size);
                        *((uint16_t *)&dl_file_data[current_chunk_len + entry_size]) = crcval;
                        current_chunk_len += entry_size + sizeof(uint16_t);
                        entry_was_sent = true;
                        //printk("crc, ccl: %d\n", current_chunk_len);

                    } else {
                        /* can't fit any more entries into this chunk */
                        break;
                    }
                }
            } while (current_chunk_len < (payloadBufferAvail));

            if (current_chunk_len > 0)
            {
                bytes_to_read = current_chunk_len; /* data log entries */
            }
        }

        /* correct the bytes_to_read if it would be past the end of the file */
        if((off + bytes_to_read) > file_len) {
#if DEBUG_CBOR_PAYLOADS
            printk("truncating bytes_to_read (off: %d, bytes_to_read: %d, file_len: %d)\n", (uint32_t)off, bytes_to_read, file_len);
#endif
            bytes_to_read = file_len - off;
        }

        /* fake the # of bytes read */
        bytes_read = bytes_to_read;
#if DEBUG_CBOR_PAYLOADS
        printk("sending off: %lld, bytes_read: %d\n", off, bytes_read);
#endif

#if DEBUG_CBOR_PAYLOADS
        // static uint8_t hex_buf[2048];
        // char *hb = hex_buf;
        // for(int i = 0; i < bytes_read; i++)
        // {
        //     hb += sprintf(hb, "%02X", dl_file_data[i]);
        // }
        // printk("%s\n", hex_buf);
#endif

        if ((bytes_read + off) >= file_len || end_transfer)
        {
            /* Encode the response. */
            err = 0;
            err |= cbor_encode_text_stringz(&ctxt->encoder, "off");
            if(err) { printk("cbor err [off]: %d\n", err); }
            err |= cbor_encode_uint(&ctxt->encoder, off);
            if(err) { printk("cbor err [offv]: %d\n", err); }
            err |= cbor_encode_text_stringz(&ctxt->encoder, "data");
            if(err) { printk("cbor err [data]: %d\n", err); }            
            err |= cbor_encode_byte_string(&ctxt->encoder, dl_file_data, bytes_read);
            if(err) { printk("cbor err [datav]: %d\n", err); }
            err |= cbor_encode_text_stringz(&ctxt->encoder, "rc");
            if(err) { printk("cbor err [rc]: %d\n", err); }
            if (bytes_read + off >= file_len)
            {
                err |= cbor_encode_int(&ctxt->encoder, MGMT_ERR_EOK);
            } else {
                err |= cbor_encode_int(&ctxt->encoder, MGMT_ERR_EINVAL); // make sure the client stops requesting more chunks
            }
            if(err) { printk("cbor err [rcv]: %d\n", err); }
            if (off == 0) {
                printk("sending len: %d\n", file_len);
                err |= cbor_encode_text_stringz(&ctxt->encoder, "len");
                if(err) { printk("cbor err [len]: %d\n", err); }
                err |= cbor_encode_uint(&ctxt->encoder, file_len);
                if(err) { printk("cbor err [lenv]: %d\n", err); }
            }

            /* sets up an interval timer to keep from setting the "hasData" flag until a timeout period */
            DataLogWasRead();
            logReadCompleted = true;

            /* signal completion of the data log download */
            DataLogReadComplete();
        }
        else
        {
            /* Encode the response. */
            err = 0;
            err |= cbor_encode_text_stringz(&ctxt->encoder, "off");
            if(err) { printk("cbor err [off]: %d\n", err); }
            err |= cbor_encode_uint(&ctxt->encoder, off);
            if(err) { printk("cbor err [offv]: %d\n", err); }
            err |= cbor_encode_text_stringz(&ctxt->encoder, "data");
            if(err) { printk("cbor err [data]: %d\n", err); }
            err |= cbor_encode_byte_string(&ctxt->encoder, dl_file_data, bytes_read);
            if(err) { printk("cbor err [datav]: %d\n", err); }
            err |= cbor_encode_text_stringz(&ctxt->encoder, "rc");
            if(err) { printk("cbor err [rc]: %d\n", err); }
            err |= cbor_encode_int(&ctxt->encoder, MGMT_ERR_EOK);
            if(err) { printk("cbor err [rcv]: %d\n", err); }
            if (off == 0) {
                printk("sending len: %d\n", file_len);
                err |= cbor_encode_text_stringz(&ctxt->encoder, "len");
                if(err) { printk("cbor err [len]: %d\n", err); }
                err |= cbor_encode_uint(&ctxt->encoder, file_len);
                if(err) { printk("cbor err [lenv]: %d\n", err); }
            }
        }

    }
    else
    {
        /* Only the response to the first download request contains the total file
        * length.
        */
        if (off == 0) {
            rc = fs_mgmt_impl_filelen(path, &file_len);
            if (rc != 0) {
                return rc;
            }
        }

        /* Read the requested chunk from the file. */
        rc = fs_mgmt_impl_read(path, off, FS_MGMT_MAX_PAYLOAD_SIZE,
                               dl_file_data, &bytes_read);
        if (rc != 0) {
            return rc;
        }

        /* Encode the response. */
        err = 0;
        err |= cbor_encode_text_stringz(&ctxt->encoder, "off");
        err |= cbor_encode_uint(&ctxt->encoder, off);
        err |= cbor_encode_text_stringz(&ctxt->encoder, "data");
        err |= cbor_encode_byte_string(&ctxt->encoder, dl_file_data, bytes_read);
        err |= cbor_encode_text_stringz(&ctxt->encoder, "rc");
        err |= cbor_encode_int(&ctxt->encoder, MGMT_ERR_EOK);
        if (off == 0) {
#if DEBUG_CBOR_PAYLOADS
            printk("sending len: %d\n", file_len);
#endif
            err |= cbor_encode_text_stringz(&ctxt->encoder, "len");
            err |= cbor_encode_uint(&ctxt->encoder, file_len);
        }
    }

    if (err != 0) {
        return MGMT_ERR_ENOMEM;
    }

    return 0;
}

/**
 * Encodes a file upload response.
 */
static int
fs_mgmt_file_upload_rsp(struct mgmt_ctxt *ctxt, int rc, unsigned long long off)
{
    CborError err;

    err = 0;
    err |= cbor_encode_text_stringz(&ctxt->encoder, "rc");
    err |= cbor_encode_int(&ctxt->encoder, rc);
    err |= cbor_encode_text_stringz(&ctxt->encoder, "off");
    err |= cbor_encode_uint(&ctxt->encoder, off);

    if (err != 0) {
        return MGMT_ERR_ENOMEM;
    }

    return 0;
}

static uint8_t ul_file_data[FS_MGMT_MAX_CBOR_BUF_SIZE];

/**
 * Command handler: fs file (write)
 */
static int
fs_mgmt_file_upload(struct mgmt_ctxt *ctxt)
{
    char file_name[FS_MGMT_PATH_SIZE + 1];
    unsigned long long len;
    unsigned long long off;
    size_t data_len;
    size_t new_off;
    int rc;

    const struct cbor_attr_t uload_attr[5] = {
        [0] = {
            .attribute = "off",
            .type = CborAttrUnsignedIntegerType,
            .addr.uinteger = &off,
            .nodefault = true
        },
        [1] = {
            .attribute = "data",
            .type = CborAttrByteStringType,
            .addr.bytestring.data = ul_file_data,
            .addr.bytestring.len = &data_len,
            .len = sizeof(ul_file_data)
        },
        [2] = {
            .attribute = "len",
            .type = CborAttrUnsignedIntegerType,
            .addr.uinteger = &len,
            .nodefault = true
        },
        [3] = {
            .attribute = "name",
            .type = CborAttrTextStringType,
            .addr.string = file_name,
            .len = sizeof(file_name)
        },
        [4] = {0},
    };

    len = ULLONG_MAX;
    off = ULLONG_MAX;
    rc = cbor_read_object(&ctxt->it, uload_attr);
    if (rc != 0 || off == ULLONG_MAX || file_name[0] == '\0') {
        return MGMT_ERR_EINVAL;
    }

    //If the file path is /lfs/time, instead of writing to filesystem, update the rtc.
    if ((strncmp(file_name, TIME_SET_FILENAME, TIME_SET_FILENAME_LEN) == 0) ||
        (strncmp(file_name, TIME_SET_TXT_FILENAME, TIME_SET_TXT_FILENAME_LEN) == 0))
    {
        if (data_len > 0)
        {
            /* Update internal clock */
            SysTime_t sysTime;
            bool success = false;
            uint32_t rxTime = strtoul(ul_file_data, NULL, 0);
            if(rxTime != ULONG_MAX)
            {
                sysTime.Seconds = rxTime;
                sysTime.SubSeconds = 0;
                success = SysTimeSet(sysTime);

                if(success)
                {    
                    set_time_src(TIME_SRC_SMP);
                }

                fs_mgmt_ctxt.len = data_len;
                fs_mgmt_ctxt.off = data_len;
            }

            if(!success)
            {
                return MGMT_ERR_EINVAL;
            }
        }
    }
    else if ((strncmp(file_name, LED_PATTERN_FILENAME, LED_PATTERN_FILENAME_LEN) == 0) ||
             (strncmp(file_name, LED_PATTERN_TXT_FILENAME, LED_PATTERN_TXT_FILENAME_LEN) == 0))
    {
        if (data_len > 0)
        {
            /* Set the specified LED pattern */
            uint32_t led_pattern = strtoul(ul_file_data, NULL, 0);
            if(led_pattern != ULONG_MAX)
            {
                if (led_pattern < LED_PATTERN_COUNT)
                {
                    led_trigger_pattern(led_pattern);
                }
            }
            else
            {
                return MGMT_ERR_EINVAL;
            }
            fs_mgmt_ctxt.len = data_len;
            fs_mgmt_ctxt.off = data_len;
        }
    }    
    else
    {
        if (off == 0)
        {
            /* Total file length is a required field in the first chunk request. */
            if (len == ULLONG_MAX)
            {
                LOG_WRN("Invalid length %llx", len);
                return MGMT_ERR_EINVAL;
            }

            fs_mgmt_ctxt.uploading = true;
            fs_mgmt_ctxt.off = 0;
            fs_mgmt_ctxt.len = len;
        }
        else
        {
            if (!fs_mgmt_ctxt.uploading)
            {
                return MGMT_ERR_EINVAL;
            }

            if (off != fs_mgmt_ctxt.off)
            {
                /* Invalid offset.  Drop the data and send the expected offset. */
                return fs_mgmt_file_upload_rsp(ctxt, MGMT_ERR_EINVAL,
                                               fs_mgmt_ctxt.off);
            }
        }

        new_off = fs_mgmt_ctxt.off + data_len;
        if (new_off > fs_mgmt_ctxt.len)
        {
            /* Data exceeds image length. */
            return MGMT_ERR_EINVAL;
        }

        if (data_len > 0)
        {
            /* Write the data chunk to the file. */
            rc = fs_mgmt_impl_write(file_name, off, ul_file_data, data_len);
            if (rc != 0)
            {
                return rc;
            }
            fs_mgmt_ctxt.off = new_off;
        }
    }

    if (fs_mgmt_ctxt.off == fs_mgmt_ctxt.len) {
        /* Upload complete. */
        fs_mgmt_ctxt.uploading = false;

        //If params.txt was uploaded, update ram copy
        if (strncmp(file_name, SETTINGS_FILENAME, SETTINGS_FILENAME_LEN) == 0)
        {
            SettingsReadFromFile();
        }    
    }


    /* Send the response. */
    return fs_mgmt_file_upload_rsp(ctxt, 0, fs_mgmt_ctxt.off);
}

void FsMgmtConnectedCb(void)
{
    logReadCompleted = false;
}

void FsMgmtDisconnectedCb(void)
{
    if(logReadCompleted)
    {
        consecutiveLogReadFailures = 0;
    }
    else
    {
        //Keep track of number of consecutive connections where log read
        //was not completed successfully. If this count hits a maximum,
        //call DataLogWasRead() to start "log interval timer" to hold off
        //and try again later (possible causes: authentication failures with remote device, unexpected disconnects)
        consecutiveLogReadFailures++;
        LOG_WRN("consecutiveLogReadFailures %d", consecutiveLogReadFailures);
        if(consecutiveLogReadFailures >= MAX_CONSECUTIVE_LOG_READ_FAILURES)
        {
            LOG_WRN("Consecutive log read failures max exceeded (%d >= %d). Start log timer...", consecutiveLogReadFailures, MAX_CONSECUTIVE_LOG_READ_FAILURES);
            DataLogWasRead();
            consecutiveLogReadFailures = 0;
        }
    }
    logReadCompleted = false;
}

/**************************************************************/
