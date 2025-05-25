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
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(nrf, LOG_LEVEL_INF);

#define PWM_PERIOD_USEC 20000

#define PAN_LEFT_USEC 2600
#define PAN_RIGHT_USEC 700
#define PAN_CENTER_USEC 1700

#define TILT_DOWN_USEC 500
#define TILT_UP_USEC 2000
#define TILT_CENTER_USEC 1000

#define CAM_PAN_STEP_USEC 200
#define CAM_TILT_STEP_USEC 100

#define CAM_PAN_MIN_USEC 800
#define CAM_PAN_CENTER_USEC 1300
#define CAM_PAN_MAX_USEC 1800

#define CAM_TILT_MIN_USEC 800
#define CAM_TILT_CENTER_USEC 1100
#define CAM_TILT_MAX_USEC 1400

static const struct pwm_dt_spec motor_pan = PWM_DT_SPEC_GET(DT_ALIAS(motor_servo_pan));
static const struct pwm_dt_spec motor_tilt = PWM_DT_SPEC_GET(DT_ALIAS(motor_servo_tilt));
static const struct pwm_dt_spec speaker = PWM_DT_SPEC_GET(DT_ALIAS(speaker_out));
static const struct pwm_dt_spec cam_pan = PWM_DT_SPEC_GET(DT_ALIAS(cam_servo_pan));
static const struct pwm_dt_spec cam_tilt = PWM_DT_SPEC_GET(DT_ALIAS(cam_servo_tilt));

static int32_t cam_pan_pos = CAM_PAN_CENTER_USEC;
static int32_t cam_tilt_pos = CAM_TILT_CENTER_USEC;

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

static bool ad_extract_rpi_msg(struct bt_data *data, void *user_data)
{
  char *mfg_buf = user_data;
  if (data->type == BT_DATA_MANUFACTURER_DATA) {
    size_t len = MIN(data->data_len, 30);
    // Skip the 2-byte manufacturer ID (e.g., 0xFFFF)
    if (data->data_len > 2) {
      memcpy(mfg_buf, data->data + 2, len - 2);
      mfg_buf[len - 2] = '\0';
    } else {
      mfg_buf[0] = '\0';  // Invalid manufacturer payload
    }
    return false;
  }
  return true;
}

void handle_command(int cmd)
{
  switch (cmd) {
  case 0: // GO
    pwm_set(motor_tilt.dev, motor_tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
    pwm_set(motor_pan.dev, motor_pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
    LOG_INF("[ACTION] GO");
    break;
  case 1: // STOP
    pwm_set(motor_tilt.dev, motor_tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_UP_USEC), 0);
    pwm_set(motor_pan.dev, motor_pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
    LOG_INF("[ACTION] STOP");
    break;
  case 2: // LEFT
    pwm_set(motor_pan.dev, motor_pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_LEFT_USEC), 0);
    pwm_set(motor_tilt.dev, motor_tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
    LOG_INF("[ACTION] LEFT");
    break;
  case 3: // RIGHT
    pwm_set(motor_pan.dev, motor_pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_RIGHT_USEC), 0);
    pwm_set(motor_tilt.dev, motor_tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
    LOG_INF("[ACTION] RIGHT");
    break;
  default:
    LOG_WRN("Unknown command received: %d", cmd);
    break;
  }
}

void handle_camera_movement(char cam_pan_buf[5], char cam_tilt_buf[5]) {
  int pan_dir = cam_pan_buf[0] - '0';
  int pan_offset = atoi(&cam_pan_buf[1]);
  int tilt_dir = cam_tilt_buf[0] - '0';
  int tilt_offset = atoi(&cam_tilt_buf[1]);

  cam_pan_pos = (pan_dir == 2) ? CAM_PAN_CENTER_USEC
                               : CAM_PAN_CENTER_USEC + ((pan_dir == 1) ? pan_offset : -pan_offset);

  cam_tilt_pos = (tilt_dir == 2) ? CAM_TILT_CENTER_USEC
                                 : CAM_TILT_CENTER_USEC + ((tilt_dir == 1) ? tilt_offset : -tilt_offset);

  pwm_set(cam_pan.dev, cam_pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(cam_pan_pos), 0);
  pwm_set(cam_tilt.dev, cam_tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(cam_tilt_pos), 0);

  LOG_INF("Cam moved to pan=%ld tilt=%ld", (long)cam_pan_pos, (long)cam_tilt_pos);
}

void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
             uint8_t type, struct net_buf_simple *ad)
{
  char mfg_buf[32] = {0};
  bt_data_parse(ad, ad_extract_rpi_msg, mfg_buf);

  if (strncmp(mfg_buf, "B1:", 3) == 0) {
    int cmd = -1;
    int int_part = 0;
    int frac_part = 0;
    char cam_pan_buf[5] = {0};  // 4 digits + null
    char cam_tilt_buf[5] = {0};

    if (sscanf(mfg_buf + 3, "%d,%d.%d,%4s,%4s",
               &cmd, &int_part, &frac_part, cam_pan_buf, cam_tilt_buf) == 5) {

      LOG_INF("Parsed BLE: cmd=%d, dist=%d.%02d, cam_pan=%s, cam_tilt=%s",
              cmd, int_part, frac_part, cam_pan_buf, cam_tilt_buf);

      // Convert distance and command
      handle_command(cmd);
      handle_distance(int_part, frac_part);
      handle_camera_movement(cam_pan_buf, cam_tilt_buf);
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

static int clamp(const struct shell *shell, int val, int min, int max, const char *label) {
  if (val < min) {
    shell_print(shell, "%s reached minimum limit (%d)", label, min);
    return min;
  }
  if (val > max) {
    shell_print(shell, "%s reached maximum limit (%d)", label, max);
    return max;
  }
  return val;
}

// Shell commands
static int cmd_simulate(const struct shell *shell, size_t argc, char **argv) {
  if (argc != 3) {
    shell_print(shell, "Usage: simulate <gesture:go|stop|left|right> <distance:float>");
    return -EINVAL;
  }

  const char *gesture = argv[1];
  float distance = strtof(argv[2], NULL);
  int cmd = -1;

  if (strcmp(gesture, "go") == 0) {
    cmd = 0;
  } else if (strcmp(gesture, "stop") == 0) {
    cmd = 1;
  } else if (strcmp(gesture, "left") == 0) {
    cmd = 2;
  } else if (strcmp(gesture, "right") == 0) {
    cmd = 3;
  } else {
    shell_print(shell, "Invalid gesture: %s", gesture);
    return -EINVAL;
  }

  handle_command(cmd);

  int int_part = (int)distance;
  int frac_part = (int)((distance - int_part) * 100);
  handle_distance(int_part, frac_part);

  shell_print(shell, "Simulating gesture %s and distance %d.%02d", gesture, int_part, frac_part);
  return 0;
}

static int cmd_tone(const struct shell *shell, size_t argc, char **argv) {
  if (argc != 2) {
    shell_print(shell, "Usage: tone <off|on|distance:float>");
    return -EINVAL;
  }

  const char *arg = argv[1];

  if (strcmp(arg, "off") == 0) {
    k_mutex_lock(&tone_ctrl.lock, K_FOREVER);
    tone_ctrl.active = false;
    k_mutex_unlock(&tone_ctrl.lock);
    shell_print(shell, "Tone disabled");
  } else if (strcmp(arg, "on") == 0) {
    k_mutex_lock(&tone_ctrl.lock, K_FOREVER);
    tone_ctrl.active = true;
    k_mutex_unlock(&tone_ctrl.lock);
    shell_print(shell, "Tone enabled");
  } else {
    float d = strtof(arg, NULL);
    int int_part = (int)d;
    int frac_part = (int)((d - int_part) * 100);
    handle_distance(int_part, frac_part);
    shell_print(shell, "Tone set for distance %d.%02d", int_part, frac_part);
  }

  return 0;
}

static int cmd_pt(const struct shell *shell, size_t argc, char **argv) {
  if (argc < 3) {
    shell_print(shell, "Usage: pt <cam|motor> <args>");
    return -EINVAL;
  }

  const char *target = argv[1];

  if (strcmp(target, "cam") == 0) {
    if (argc != 4) {
      shell_print(shell, "Usage: pt cam <pan|tilt> <left|right|center|up|down>");
      return -EINVAL;
    }

    const char *axis = argv[2];
    const char *dir = argv[3];
    int pulse = 0;

    if (strcmp(axis, "pan") == 0) {
      if (strcmp(dir, "left") == 0) {
        cam_pan_pos -= CAM_PAN_STEP_USEC;
      } else if (strcmp(dir, "right") == 0) {
        cam_pan_pos += CAM_PAN_STEP_USEC;
      } else if (strcmp(dir, "center") == 0) {
        cam_pan_pos = CAM_PAN_CENTER_USEC;
      } else {
        shell_print(shell, "Invalid direction for pan: %s", dir);
        return -EINVAL;
      }
      cam_pan_pos = clamp(shell, cam_pan_pos, CAM_PAN_MIN_USEC, CAM_PAN_MAX_USEC, "Cam pan");
      pulse = cam_pan_pos;
      pwm_set(cam_pan.dev, cam_pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(pulse), 0);
    } else if (strcmp(axis, "tilt") == 0) {
      if (strcmp(dir, "up") == 0) {
        cam_tilt_pos -= CAM_TILT_STEP_USEC;
      } else if (strcmp(dir, "down") == 0) {
        cam_tilt_pos += CAM_TILT_STEP_USEC;
      } else if (strcmp(dir, "center") == 0) {
        cam_tilt_pos = CAM_TILT_CENTER_USEC;
      } else {
        shell_print(shell, "Invalid direction for tilt: %s", dir);
        return -EINVAL;
      }
      cam_tilt_pos = clamp(shell, cam_tilt_pos, CAM_TILT_MIN_USEC, CAM_TILT_MAX_USEC, "Cam tilt");
      pulse = cam_tilt_pos;
      pwm_set(cam_tilt.dev, cam_tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(pulse), 0);
    } else {
      shell_print(shell, "Invalid axis: %s", axis);
      return -EINVAL;
    }

    shell_print(shell, "Moved cam %s to %s -> pulse = %d", axis, dir, pulse);
  } else if (strcmp(target, "motor") == 0) {
    const char *action = argv[2];
    if (strcmp(action, "go") == 0) {
      handle_command(0);
    } else if (strcmp(action, "stop") == 0) {
      handle_command(1);
    } else if (strcmp(action, "left") == 0) {
      handle_command(2);
    } else if (strcmp(action, "right") == 0) {
      handle_command(3);
    } else {
      shell_print(shell, "Invalid motor action: %s", action);
      return -EINVAL;
    }
    shell_print(shell, "Moved motor for action %s", action);
  } else {
    shell_print(shell, "Unknown target: %s", target);
    return -EINVAL;
  }

  return 0;
}

SHELL_CMD_REGISTER(simulate, NULL, "Simulate <gesture> <distance>", cmd_simulate);
SHELL_CMD_REGISTER(tone, NULL, "Tone control or simulate distance tone", cmd_tone);
SHELL_CMD_REGISTER(pt, NULL, "Pan-tilt control for cam or motor", cmd_pt);

void main(void)
{
  if (!device_is_ready(motor_pan.dev) || !device_is_ready(motor_tilt.dev)
      || !device_is_ready(cam_pan.dev) || !device_is_ready(cam_tilt.dev)) {
    printk("Error: Servo device(s) not ready\n");
    return;
  }

  // Recenter servos on boot
  pwm_set(motor_pan.dev, motor_pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
  pwm_set(motor_tilt.dev, motor_tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_CENTER_USEC), 0);
  pwm_set(cam_pan.dev, cam_pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(CAM_PAN_CENTER_USEC), 0);
  pwm_set(cam_tilt.dev, cam_tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(CAM_TILT_CENTER_USEC), 0);

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
                  (k_thread_entry_t)tone_thread_fn, NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
}