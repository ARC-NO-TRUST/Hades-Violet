#ifndef BT_MOBILE_H
#define BT_MOBILE_H

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/shell/shell.h>

// Stack sizes and priorities
#define BT_STACK_SIZE 4096
#define BT_PRIORITY 5
#define BLE_BUFFER_LEN 20

// Message queue Struct define
struct accel_msg {
	int x;
    int y;
    int z;
};

extern struct k_msgq accel_message_queue;

void bt_ready(int err);
int bt_mobile_start_ad(void);
void mobile_bluetooth_thread_fn(void *p1, void *p2, void *p3);
void mobile_bluetooth_init(void);


#endif