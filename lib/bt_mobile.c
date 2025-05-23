#include "bt_mobile.h"
#include "accelerometer.h"
#include <string.h>
#include <stddef.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/shell/shell.h>

// Threads and stacks
static struct k_thread bt_thread;
K_THREAD_STACK_DEFINE(bt_stack, BT_STACK_SIZE);
K_MSGQ_DEFINE(accel_message_queue, sizeof(struct accel_msg), 10, 1);

char transmit_buffer[BLE_BUFFER_LEN] = "[INIT MSG]";
size_t buffer_len = sizeof(transmit_buffer);

struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "TY", 2),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, transmit_buffer, 20),
};

void bt_ready(int err)
{
	if (err) {
		printk("[BT READY] BT READY failed (err %d)\n", err);
		return;
	}
	/* Start advertising */
	printk("[BT READY] BT READY successful\n");
}


int bt_mobile_start_ad(void) {
	int err = bt_le_adv_start(BT_LE_ADV_PARAM(
								BT_LE_ADV_OPT_USE_IDENTITY,
								BT_GAP_ADV_FAST_INT_MIN_2,
								BT_GAP_ADV_FAST_INT_MAX_2,
								NULL),
							ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("[MOBILE AD] Failed to start advertising (err %d)\n", err);
        return err;
    }
	return 0;
}

void print_bt_address(void)
{
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t count = CONFIG_BT_ID_MAX;

    bt_id_get(addrs, &count);
    if (count == 0) {
        printk("[BT MAC] Failed to get local address\n");
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&addrs[0], addr_str, sizeof(addr_str));
    printk("[BT MAC] Local Bluetooth Address: %s\n", addr_str);
}


void mobile_bluetooth_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

	int err;

	err = bt_enable(bt_ready);
	if (err) {
		printk("[BT ENABLE] Bluetooth init failed\n");
	} else {
        printk("[BT ENABLE] Bluetooth init success\n");
    }

	(void)bt_mobile_start_ad();

	// int gesture = 0;
	// int distance_cycle_index = 0;
	// const int max_index = 8;

	while (1) {
		struct accel_msg msg;
		
		if (k_msgq_get(&accel_message_queue, &msg, K_FOREVER) == 0) {
			snprintf(transmit_buffer, sizeof(transmit_buffer), 
				"M1:%03d,%03d,%03d", msg.x, msg.y, msg.z);
			// int distance_int = (distance_cycle_index / 2);
			// int distance_frac = (distance_cycle_index % 2 == 0) ? 20 : 70;

			// snprintf(transmit_buffer, sizeof(transmit_buffer), 
			// 	"B1:%01d,%02d.%02d", gesture, distance_int, distance_frac);
		
			ad[2].data = transmit_buffer;
			ad[2].data_len = 20;

			int err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
			if (err) {
				printk("[BT MOBILE] Failed to update advertising data (err %d) - %s\n", err, transmit_buffer);
			} else {
				printk("[BT MOBILE] Update Successful: %s \n", transmit_buffer);
			}

			// gesture = (gesture + 1) % 4;
			// distance_cycle_index = (distance_cycle_index + 1) % (max_index + 1);
		}
		k_sleep(K_MSEC(200));
	}
}

void mobile_bluetooth_init(void) {
	k_thread_create(&bt_thread, bt_stack,
                K_THREAD_STACK_SIZEOF(bt_stack),
                mobile_bluetooth_thread_fn, NULL, NULL, NULL,
                BT_PRIORITY, 0, K_NO_WAIT);

}



