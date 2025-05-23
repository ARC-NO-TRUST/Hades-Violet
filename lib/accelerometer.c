#include "accelerometer.h"
#include "bt_mobile.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <math.h>

#define DEBUG

static const struct device *accel_dev  = DEVICE_DT_GET(DT_ALIAS(accel0));
K_THREAD_STACK_DEFINE(accel_stack, STACK_SIZE);
static struct k_thread accel_thread_data;

static void accel_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

  	struct sensor_value x, y, z;

  	while (1) {
    	if (sensor_sample_fetch(accel_dev) == 0 &&
			sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &x) == 0 &&
			sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &y) == 0 &&
			sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &z) == 0) {

			float xf = sensor_value_to_double(&x);
			float yf = sensor_value_to_double(&y);
			float zf = sensor_value_to_double(&z);

			#ifdef DEBUG
				int x_int = (int)xf;
				int x_frac = (int)((fabsf(xf - x_int)) * 100.0f + 0.5f);
				int y_int = (int)yf;
				int y_frac = (int)((fabsf(yf - y_int)) * 100.0f + 0.5f);
				int z_int = (int)zf;
				int z_frac = (int)((fabsf(zf - z_int)) * 100.0f + 0.5f);
			
				printk("[ACC] Accel X=%d.%02d Y=%d.%02d Z=%d.%02d\n",
					x_int, x_frac,
					y_int, y_frac,
					z_int, z_frac);
			#endif

			struct accel_msg msg = {
				.x = (int)roundf(xf * 100.0f),
				.y = (int)roundf(yf * 100.0f),
				.z = (int)roundf(zf * 100.0f),
			};

			printk("[ACC RAW] x=%d y=%d z=%d\n", msg.x, msg.y, msg.z);

			k_msgq_put(&accel_message_queue, &msg, K_NO_WAIT);
    	}
    	k_sleep(K_MSEC(POLL_INTERVAL_MS));
  	}
}


int accelerometer_init(void)
{
	if (!device_is_ready(accel_dev)) {
		printk("[ACC][ERR]Accelerometer device not ready\n");
		return -1;
	}
	k_thread_create(&accel_thread_data, accel_stack,
					K_THREAD_STACK_SIZEOF(accel_stack),
					accel_thread, NULL, NULL, NULL,
					THREAD_PRIORITY, 0, K_NO_WAIT);
	return 0;
}
