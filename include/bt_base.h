#ifndef BT_BASE_H
#define BT_BASE_H

#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>


// Constants
#define BASE_SCAN_STACK_SIZE 2048
#define BASE_SCAN_THREAD_PRIORITY 5
#define NAME_THINGY "C8:AE:54:01:AC:A9 (random)"
#define RASP_PI "D8:3A:DD:CB:40:27 (public)"

// External message queue declaration
extern struct k_msgq base_recv_message_queue;

// Function declarations
int bt_base_start_scan(void);
void start_mobile_scan_thread(void);
void base_scan_thread_fn(void *p1, void *p2, void *p3);
void start_mobile_scan_thread(void);
void bt_base_print_address(void);

#endif // BT_BASE_H