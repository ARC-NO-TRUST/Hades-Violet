#include "bt_base.h"
#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/shell/shell.h>


void bt_ready(int err)
{
	if (err) {
		printk("[MAIN] Bluetooth init failed (err %d)\n", err);
		return;
	}
	printk("[MAIN] Bluetooth init success\n");
}


int main(void)
{
	int err;

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("[MAIN] Bluetooth init failed (err %d)\n", err);
	} else {
        printk("[MAIN] Bluetooth init success\n");
    }
    (void)start_mobile_scan_thread();
 
	return 0;
}