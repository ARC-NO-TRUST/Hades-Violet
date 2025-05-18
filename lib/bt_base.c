#include "bt_base.h"
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#define DEBUG

static struct k_thread base_scan_thread_data;

K_THREAD_STACK_DEFINE(base_scan_stack_area, BASE_SCAN_STACK_SIZE);

K_MSGQ_DEFINE(base_recv_message_queue, 30, 10, 1);

static bool ad_extract_msg(struct bt_data *data, void *user_data)
{
    char *mfg_buf = user_data;
    const uint8_t* payload = data->data;

    printk("Found Manufacturer Data, %u bytes: %*ph\n", data->data_len, data->data_len, data->data);

    if (data->type == BT_DATA_MANUFACTURER_DATA) {
        size_t len = MIN(data->data_len, 30);
        memcpy(mfg_buf, payload+2, len - 2);
        mfg_buf[len - 2] = '\0';
        return false;
    }

    return true;
}

static bool ad_find_name(struct bt_data *data, void *user_data)
{
    char *name_buf = user_data;

    if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
        size_t len = MIN(data->data_len, 31);
        memcpy(name_buf, data->data, len);
        name_buf[len] = '\0';
        return false; // stop parsing, name found
    }

    return true; // continue parsing
}

static void bt_base_process_data(char *msg) {
    int ret = k_msgq_put(&base_recv_message_queue, msg, K_NO_WAIT);
    if (ret != 0) {
        printk("[BASE][ERROR] Failed to enqueue message (err %d)\n", ret);
    } else {
        printk("[BASE][INFO] Message enqueued: %s\n", msg);
    }
}

static bool ad_debug_dump(struct bt_data *data, void *user_data)
{
    printk("AD type 0x%02X (%u bytes): %*ph\n",
           data->type, data->data_len,
           data->data_len, data->data);
    return true; // keep parsing
}


static void bt_base_scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                         uint8_t type, struct net_buf_simple *ad)
{
	char dev[BT_ADDR_LE_STR_LEN];
    char msg[32] = {0};
    

	bt_addr_le_to_str(addr, dev, sizeof(dev));

    #ifdef DEBUG
        printk("[BASE][DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
            dev, type, ad->len, rssi);
    #endif

    if (strcmp(dev, NAME_THINGY) == 0) {
        bt_data_parse(ad, ad_extract_msg, msg);
        bt_base_process_data(msg);
    } else if (strcmp(dev, RASP_PI) == 0) {
        bt_data_parse(ad, ad_debug_dump, NULL);
        bt_data_parse(ad, ad_extract_msg, msg);
        printk("[BASE][DEVICE] RASPBERRY PI Found - PAYLOAD: %s\n", msg);
    }

}

int bt_base_start_scan(void)
{
    struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL ,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};
    
    int err;
    err = bt_le_scan_start(&scan_param, bt_base_scan_cb);
    if (err) {
        printk("[CENTRAL] Scanning failed to start (err %d)\n", err);
        return -1;
    }
    return 0;
}

void base_scan_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("[CENTRAL]Starting Central Thread\n");
    (void)bt_base_start_scan();
    printk("[CENTRAL]Exiting Central Thread\n");

    return;
}

void start_mobile_scan_thread(void) {
    k_thread_create(&base_scan_thread_data, base_scan_stack_area,
                K_THREAD_STACK_SIZEOF(base_scan_stack_area),
                base_scan_thread_fn,
                NULL, NULL, NULL,
                BASE_SCAN_THREAD_PRIORITY, 0, K_NO_WAIT);
}


