#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nrf, LOG_LEVEL_INF);

#define PWM_PERIOD_USEC 20000

#define PAN_LEFT_USEC   2600
#define PAN_RIGHT_USEC  700
#define PAN_CENTER_USEC 1700

#define TILT_DOWN_USEC  500
#define TILT_UP_USEC    2000
#define TILT_CENTER_USEC 1000

static const struct pwm_dt_spec pan = PWM_DT_SPEC_GET(DT_ALIAS(servo_pan));
static const struct pwm_dt_spec tilt = PWM_DT_SPEC_GET(DT_ALIAS(servo_tilt));
static const struct pwm_dt_spec speaker = PWM_DT_SPEC_GET(DT_ALIAS(speaker_out));

#define STACK_SIZE 2048
#define PRIORITY 5

K_THREAD_STACK_DEFINE(bt_scan_stack, STACK_SIZE);
static struct k_thread bt_scan_thread_data;

struct bt_le_scan_param scan_param = {
    .type     = BT_HCI_LE_SCAN_PASSIVE,
    .options  = BT_LE_SCAN_OPT_NONE,
    .interval = 0x0010,
    .window   = 0x0010,
};

static bool ad_find_mfg(struct bt_data *data, void *user_data)
{
  char *mfg_buf = user_data;
  if (data->type == BT_DATA_MANUFACTURER_DATA) {
    size_t len = MIN(data->data_len, 15);
    memcpy(mfg_buf, data->data, len);
    mfg_buf[len] = '\0';
    return false;
  }
  return true;
}

void play_tone(uint32_t freq_hz, uint32_t duration_ms)
{
  if (!device_is_ready(speaker.dev)) {
    printk("Speaker PWM device not ready\n");
    return;
  }

  uint32_t period = USEC_PER_SEC / freq_hz;
  pwm_set(speaker.dev, speaker.channel, period, period / 2, 0);
  k_msleep(duration_ms);
  pwm_set(speaker.dev, speaker.channel, 0, 0, 0);  // Stop PWM
}

void handle_command(int cmd)
{
  switch (cmd) {
  case 0: // GO
    pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
    pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
//    play_tone(1000, 300);
    LOG_INF("[ACTION] GO");
    break;
  case 1: // STOP
    pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_UP_USEC), 0);
    pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
//    play_tone(1000, 300);
    LOG_INF("[ACTION] STOP");
    break;
  case 2: // LEFT
    pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_LEFT_USEC), 0);
    pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
//    play_tone(1000, 300);
    LOG_INF("[ACTION] LEFT");
    break;
  case 3: // RIGHT
    pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_RIGHT_USEC), 0);
    pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
//    play_tone(1000, 300);
    LOG_INF("[ACTION] RIGHT");
    break;
  default:
    LOG_WRN("Unknown command received: %d", cmd);
    break;
  }
}

void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
             uint8_t type, struct net_buf_simple *ad)
{
  char mfg_buf[16] = {0};
  bt_data_parse(ad, ad_find_mfg, mfg_buf);

  if (strncmp(mfg_buf, "B1:", 3) == 0) {
    int cmd = -1;
    int int_part = 0;
    int frac_part = 0;

    if (sscanf(mfg_buf + 3, "%d,%d.%d", &cmd, &int_part, &frac_part) == 3) {
      LOG_INF("Received B1 command: %d, value: %d.%02d", cmd, int_part, frac_part);
      handle_command(cmd);
    } else {
      LOG_WRN("Malformed B1 packet: %s", mfg_buf);
    }
  }
}

void bt_scan_thread_fn(void)
{
  int err = bt_le_scan_start(&scan_param, scan_cb);

  if (err && err != -EALREADY) {
    LOG_ERR("[NRF] Failed to start scan: %d", err);
  } else {
    LOG_INF("[NRF] Scan started.");
  }
}

void main(void)
{
  if (!device_is_ready(pan.dev) || !device_is_ready(tilt.dev)) {
    printk("Error: Servo device(s) not ready\n");
    return;
  }

  // Recenter servos on boot
  pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
  pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_CENTER_USEC), 0);

  printk("Pan-tilt system ready. Waiting for Bluetooth packets...\n");

  int err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed: %d", err);
    return;
  }

  k_thread_create(&bt_scan_thread_data, bt_scan_stack, STACK_SIZE,
                  (k_thread_entry_t)bt_scan_thread_fn,
                  NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
}