#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "ult.h"

LOG_MODULE_REGISTER(base_node, LOG_LEVEL_INF);

void main(void)
{
  LOG_INF("Starting ultrasonic + Bluetooth threads...");
  start_ultrasonic_thread();
  start_bluetooth_thread();
}