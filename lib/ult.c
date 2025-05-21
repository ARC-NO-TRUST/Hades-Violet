#include "ult.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(ult, LOG_LEVEL_INF);

// Stack sizes and priorities
#define ULTRA_STACK_SIZE 4096
#define BT_STACK_SIZE 4096
#define ULTRA_PRIORITY 5
#define BT_PRIORITY 5

// Threads and stacks
static struct k_thread ultra_thread;
static struct k_thread bt_thread;
K_THREAD_STACK_DEFINE(ultra_stack, ULTRA_STACK_SIZE);
K_THREAD_STACK_DEFINE(bt_stack, BT_STACK_SIZE);

// Shared message queue
K_MSGQ_DEFINE(json_msgq, MSGQ_STRLEN, MSGQ_SIZE, 4); // 4-byte alignment

// Ultrasonic thread
static void ultrasonic_thread(void *arg1, void *arg2, void *arg3)
{
  const struct device *dev = DEVICE_DT_GET(DT_ALIAS(ultrasonic));
  struct sensor_value distance;

  if (!device_is_ready(dev)) {
    LOG_ERR("HC-SR04 device not ready");
    return;
  }

  while (1) {
    int fetch_ret = sensor_sample_fetch(dev);
    if (fetch_ret < 0) {
      LOG_ERR("Echo signal was not received or fetch failed");
      k_sleep(K_MSEC(1500));
      continue;
    }

    if (sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &distance) < 0) {
      LOG_ERR("Failed to get distance");
      k_sleep(K_MSEC(1500));
      continue;
    }

    float dist_m = distance.val1 + distance.val2 / 1000000.0f;

    // Ignore values too great - invalid
    if (dist_m > 10.0f) {
      LOG_WRN("Ignoring distance too large (%d.%02dm)", distance.val1, distance.val2);
      k_sleep(K_MSEC(1500));
      continue;
    }

    int fractional = (distance.val2 + 5000) / 10000;
    char dist_buf[MSGQ_STRLEN];
    snprintf(dist_buf, sizeof(dist_buf), "U1:%d.%02d", distance.val1, fractional);

    if (k_msgq_put(&json_msgq, dist_buf, K_NO_WAIT) != 0) {
      LOG_WRN("Message queue full, dropping message");
    }

    k_sleep(K_MSEC(1500));
  }
}

// Bluetooth thread
static void bluetooth_thread(void *arg1, void *arg2, void *arg3)
{
  int err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed: %d", err);
    return;
  }
  LOG_INF("Bluetooth initialized");

  char recv_buf[MSGQ_STRLEN];

  while (1) {
    k_msgq_get(&json_msgq, &recv_buf, K_FOREVER);

    const size_t len = strlen(recv_buf);

    struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, strlen(CONFIG_BT_DEVICE_NAME)),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, recv_buf, len),
    };

    bt_le_adv_stop();  // stop previous advertising session
    err = bt_le_adv_start(BT_LE_ADV_PARAM(
                              BT_LE_ADV_OPT_USE_IDENTITY,
                              BT_GAP_ADV_FAST_INT_MIN_2,
                              BT_GAP_ADV_FAST_INT_MAX_2,
                              NULL),
                          ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
      LOG_ERR("bt_le_adv_start failed: %d", err);
    } else {
      LOG_INF("Advertising distance: %sm", recv_buf);
    }

    k_sleep(K_MSEC(1500));
  }
}

void start_ultrasonic_thread(void)
{
  k_thread_create(&ultra_thread, ultra_stack, ULTRA_STACK_SIZE,
                  ultrasonic_thread, NULL, NULL, NULL,
                  ULTRA_PRIORITY, 0, K_NO_WAIT);
}

void start_bluetooth_thread(void)
{
  k_thread_create(&bt_thread, bt_stack, BT_STACK_SIZE,
                  bluetooth_thread, NULL, NULL, NULL,
                  BT_PRIORITY, 0, K_NO_WAIT);
}
