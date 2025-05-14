#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/init.h>

#define PWM_PERIOD_USEC 20000

#define PAN_LEFT_USEC   2700
#define PAN_RIGHT_USEC  700
#define PAN_CENTER_USEC 1700

#define TILT_DOWN_USEC  500
#define TILT_UP_USEC    2000
#define TILT_CENTER_USEC 1000

static const struct pwm_dt_spec pan = PWM_DT_SPEC_GET(DT_ALIAS(servo_pan));
static const struct pwm_dt_spec tilt = PWM_DT_SPEC_GET(DT_ALIAS(servo_tilt));

static int cmd_left(const struct shell *sh, size_t argc, char **argv)
{
  pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_LEFT_USEC), 0);
  pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
  shell_print(sh, "Pan left");
  return 0;
}

static int cmd_right(const struct shell *sh, size_t argc, char **argv)
{
  pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_RIGHT_USEC), 0);
  pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
  shell_print(sh, "Pan right");
  return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
  pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_UP_USEC), 0);
  pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
  shell_print(sh, "Tilt up");
  return 0;
}

static int cmd_go(const struct shell *sh, size_t argc, char **argv)
{
  pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_DOWN_USEC), 0);
  pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
  shell_print(sh, "Tilt down");
  return 0;
}

SHELL_CMD_REGISTER(left, NULL, "Pan left", cmd_left);
SHELL_CMD_REGISTER(right, NULL, "Pan right", cmd_right);
SHELL_CMD_REGISTER(go, NULL, "Tilt down", cmd_go);
SHELL_CMD_REGISTER(stop, NULL, "Tilt up", cmd_stop);

void main(void)
{
  if (!device_is_ready(pan.dev) || !device_is_ready(tilt.dev)) {
    printk("Error: Servo device(s) not ready\n");
    return;
  }

  // Recenter on reset
  pwm_set(pan.dev, pan.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PAN_CENTER_USEC), 0);
  pwm_set(tilt.dev, tilt.channel, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(TILT_CENTER_USEC), 0);

  printk("Pan-tilt system ready. Type commands: left, right, go, stop\n");
}
