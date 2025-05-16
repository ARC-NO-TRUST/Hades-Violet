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
K_THREAD_STACK_DEFINE(tone_stack_area, STACK_SIZE);
static struct k_thread tone_thread_data;

struct tone_control {
  struct k_mutex lock;
  float current_distance;
  bool active;
};

static struct tone_control tone_ctrl = {
    .lock = Z_MUTEX_INITIALIZER(tone_ctrl.lock),
    .current_distance = 0.0f,
    .active = false
};

void play_tone(uint32_t freq_hz, uint32_t duration_ms)
{
  if (!device_is_ready(speaker.dev)) {
    printk("Speaker PWM not ready\n");
    return;
  }

  uint32_t period = 1000000 / freq_hz;
  uint32_t pulse = period / 100;

  int ret = pwm_set_dt(&speaker, PWM_USEC(period), PWM_USEC(pulse));
  if (ret < 0) {
    printk("PWM set failed: %d\n", ret);
    return;
  }

  k_msleep(duration_ms);
  pwm_set_dt(&speaker, 0, 0);  // Stop tone
}

void tone_thread_fn(void *arg1, void *arg2, void *arg3)
{
  while (1) {
    k_mutex_lock(&tone_ctrl.lock, K_FOREVER);

    float d = tone_ctrl.current_distance;
    bool should_play = tone_ctrl.active;

    k_mutex_unlock(&tone_ctrl.lock);

    if (!should_play || d >= 2.0f) {
      k_msleep(200);
      continue;
    }

    uint32_t gap_duration = 0;

    if (d < 0.5f) {
      gap_duration = 100;
    } else if (d < 1.0f) {
      gap_duration = 500;
    } else if (d < 2.0f) {
      gap_duration = 1000;
    }

    play_tone(1000, 200); // Always beep for 200 ms
    k_msleep(gap_duration); // Wait before next beep
  }
}

void handle_distance(int int_part, int frac_part)
{
  float distance = int_part + (frac_part / 100.0f);

  k_mutex_lock(&tone_ctrl.lock, K_FOREVER);
  tone_ctrl.current_distance = distance;
  tone_ctrl.active = true;
  k_mutex_unlock(&tone_ctrl.lock);

  LOG_INF("Tracking proximity: %d.%02d m", int_part, frac_part);
}

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

void handle_command(int cmd)
{
  switch (cmd) {
  case 0: // GO
    pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
    pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
    LOG_INF("[ACTION] GO");
    break;
  case 1: // STOP
    pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_UP_USEC), 0);
    pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
    LOG_INF("[ACTION] STOP");
    break;
  case 2: // LEFT
    pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_LEFT_USEC), 0);
    pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
    LOG_INF("[ACTION] LEFT");
    break;
  case 3: // RIGHT
    pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_RIGHT_USEC), 0);
    pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
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
      handle_distance(int_part, frac_part);
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

  printk("Pan-tilt system ready\n");

  int err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed: %d", err);
    return;
  }

  k_thread_create(&bt_scan_thread_data, bt_scan_stack, STACK_SIZE,
                  (k_thread_entry_t)bt_scan_thread_fn,
                  NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
  k_thread_create(&tone_thread_data, tone_stack_area, K_THREAD_STACK_SIZEOF(tone_stack_area),
                  tone_thread_fn, NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
}