/*
 * main.c - 02_GY521_MPU6050 (core0 / procpu image)
 *
 * Architecture (dual-core AMP via IPM, change-triggered + heartbeat)
 * ---------------------------------------------------------------------
 *   [mpu6050_thread] 100ms -> reads accel XYZ, gyro XYZ, die temperature
 *                           -> sensor_hub_update(...)
 *                           -> if accel vector magnitude moves >=0.15g
 *                              OR gyro vector magnitude moves >=15 deg/s
 *                              since the last value actually sent: push now
 *
 *   [heartbeat_thread] 1000ms -> push now, unconditionally
 *
 * Unlike Lab 01 (two sensors sharing one I2C0 bus), only one thread
 * here ever touches I2C0, so there is no bus-contention concern and no
 * bus-serializing mutex is needed.
 *
 * Address auto-detect (0x68 vs 0x69)
 * -----------------------------------
 * The GY-521 module's I2C address depends on its AD0 pin: 0x68 if
 * AD0 is low/floating, 0x69 if tied to VCC. Rather than hardcoding
 * one, the overlay declares BOTH as separate devicetree nodes; the
 * mpu6050 driver's init() reads the chip's WHO_AM_I register and
 * fails (device_is_ready() == false) for whichever address has no
 * real hardware behind it. mpu6050_select(), called from
 * mpu6050_thread_fn's own startup (not from main() - see that
 * function's comment for why), picks whichever candidate actually
 * came up ready.
 *
 * Both "push now" paths funnel through push_snapshot_to_core1(), which
 * always sends the FULL current reading (all 6 axes + temperature),
 * and is itself serialized by g_send_lock so a threshold trigger and
 * the heartbeat can never call ipm_send() at the same instant.
 *
 * core0 owns I2C0 (MPU6050). core1 owns I2C1 (SSD1306) - see the
 * design doc for why the two cores never share a peripheral.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "sensor_hub.h"
#include "ipm_protocol.h"

LOG_MODULE_REGISTER(app_procpu, LOG_LEVEL_INF);

/* Devicetree handles - see boards/esp32s3_devkitc_esp32s3_procpu.overlay.
 * Both candidate addresses are declared there; exactly one of these
 * two will actually initialize successfully depending on how the
 * module's AD0 pin is wired (see mpu6050_select() below).
 */
static const struct device *dev_mpu6050_68 = DEVICE_DT_GET(DT_NODELABEL(mpu6050_68));
static const struct device *dev_mpu6050_69 = DEVICE_DT_GET(DT_NODELABEL(mpu6050_69));
static const struct device *dev_mpu6050; /* selected at boot - see mpu6050_select() */
static const struct device *dev_i2c0    = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct device *dev_ipm     = DEVICE_DT_GET(DT_NODELABEL(ipm0));

#define SENSOR_PERIOD_MS      100
#define HEARTBEAT_PERIOD_MS   1000 /* must stay well under core1's 2 s link timeout */
#define ACCEL_THRESHOLD_G     0.15f
#define GYRO_THRESHOLD_DPS    15.0f

/* Unit conversions - Zephyr's mpu6050 driver reports accel in m/s^2
 * and gyro in rad/s (SI units); this lab displays the more intuitive
 * g and deg/s instead.
 */
#define STANDARD_GRAVITY_MS2  9.80665f
#define RAD_TO_DEG            57.29578f

/* Serializes the act of pushing an IPM message - the sensor thread's
 * threshold trigger and the heartbeat thread can decide to push at
 * nearly the same instant, and ipm_send() should not be called
 * concurrently.
 */
K_MUTEX_DEFINE(g_send_lock);

static void push_snapshot_to_core1(const char *reason)
{
	struct sensor_snapshot snap;

	sensor_hub_get_snapshot(&snap);

	struct ipm_sensor_payload payload = {
		.accel_x_g = snap.accel_x_g,
		.accel_y_g = snap.accel_y_g,
		.accel_z_g = snap.accel_z_g,
		.gyro_x_dps = snap.gyro_x_dps,
		.gyro_y_dps = snap.gyro_y_dps,
		.gyro_z_dps = snap.gyro_z_dps,
		.temp_c = snap.temp_c,
		.valid = snap.valid,
	};

	k_mutex_lock(&g_send_lock, K_FOREVER);
	/* wait=0: don't block if the mailbox is momentarily busy. A
	 * dropped push is not fatal here - either the next threshold
	 * crossing or the next heartbeat will retry with fresher data.
	 */
	int ret = ipm_send(dev_ipm, 0, IPM_SENSOR_CHANNEL, &payload, sizeof(payload));
	k_mutex_unlock(&g_send_lock);

	if (ret) {
		LOG_WRN("ipm_send failed (%s): %d", reason, ret);
	} else {
		LOG_DBG("ipm_send ok (%s)", reason);
	}
}

/* Picks whichever candidate address actually has hardware behind it.
 * Called from mpu6050_thread_fn's own startup, NOT from main() - this
 * matters because K_THREAD_DEFINE threads with a 0 ms delay can start
 * running before main() does, so relying on main() to pick the device
 * first would be a race. Doing the pick inside the very thread that
 * needs it removes that race entirely.
 */
static const struct device *mpu6050_select(void)
{
	if (device_is_ready(dev_mpu6050_68)) {
		LOG_INF("MPU6050 found at 0x68 (AD0 low/floating)");
		return dev_mpu6050_68;
	}
	if (device_is_ready(dev_mpu6050_69)) {
		LOG_INF("MPU6050 found at 0x69 (AD0 high)");
		return dev_mpu6050_69;
	}
	LOG_ERR("MPU6050 not found at 0x68 or 0x69 - check wiring");
	return NULL;
}

/* ---------------------------------------------------------------------
 * MPU6050 thread - accel + gyro + temperature, every 100 ms
 * ------------------------------------------------------------------- */
static void mpu6050_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	dev_mpu6050 = mpu6050_select();

	if (dev_mpu6050 == NULL) {
		/* Nothing responded at either address. Don't spin - park
		 * this thread so the rest of the app (heartbeat, core1's
		 * link indicator) keeps running and honestly reports
		 * "no valid data" instead of crashing on a NULL device.
		 */
		while (1) {
			k_sleep(K_SECONDS(5));
		}
	}

	while (1) {
		struct sensor_value accel[3], gyro[3], temp;
		int ret = sensor_sample_fetch(dev_mpu6050);

		if (ret == 0) {
			ret = sensor_channel_get(dev_mpu6050, SENSOR_CHAN_ACCEL_XYZ, accel);
			ret |= sensor_channel_get(dev_mpu6050, SENSOR_CHAN_GYRO_XYZ, gyro);
			ret |= sensor_channel_get(dev_mpu6050, SENSOR_CHAN_DIE_TEMP, &temp);
		}

		if (ret == 0) {
			/* Driver units: accel in m/s^2, gyro in rad/s,
			 * temperature already in Celsius.
			 */
			float ax_g = sensor_value_to_double(&accel[0]) / STANDARD_GRAVITY_MS2;
			float ay_g = sensor_value_to_double(&accel[1]) / STANDARD_GRAVITY_MS2;
			float az_g = sensor_value_to_double(&accel[2]) / STANDARD_GRAVITY_MS2;
			float gx_dps = sensor_value_to_double(&gyro[0]) * RAD_TO_DEG;
			float gy_dps = sensor_value_to_double(&gyro[1]) * RAD_TO_DEG;
			float gz_dps = sensor_value_to_double(&gyro[2]) * RAD_TO_DEG;
			float temp_c = sensor_value_to_double(&temp);

			sensor_hub_update(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps,
					   temp_c, true);

			bool accel_moved = sensor_hub_accel_exceeds_threshold(
				ax_g, ay_g, az_g, ACCEL_THRESHOLD_G);
			bool gyro_moved = sensor_hub_gyro_exceeds_threshold(
				gx_dps, gy_dps, gz_dps, GYRO_THRESHOLD_DPS);

			if (accel_moved) {
				push_snapshot_to_core1("accel-threshold");
			} else if (gyro_moved) {
				/* Only push once per cycle even if both
				 * thresholds trip at once - the payload
				 * always carries every value anyway.
				 */
				push_snapshot_to_core1("gyro-threshold");
			}
		} else {
			LOG_WRN("MPU6050 read failed: %d", ret);
			sensor_hub_update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
		}

		k_sleep(K_MSEC(SENSOR_PERIOD_MS));
	}
}

/* ---------------------------------------------------------------------
 * Heartbeat thread - keeps core1's link indicator alive when nothing
 * has crossed a threshold recently
 * ------------------------------------------------------------------- */
static void heartbeat_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		push_snapshot_to_core1("heartbeat");
		k_sleep(K_MSEC(HEARTBEAT_PERIOD_MS));
	}
}

/* ---------------------------------------------------------------------
 * Thread definitions - static allocation, auto-start at boot
 * ------------------------------------------------------------------- */
K_THREAD_DEFINE(mpu6050_tid, 1024, mpu6050_thread_fn, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(heartbeat_tid, 1024, heartbeat_thread_fn, NULL, NULL, NULL, 6, 0, 0);

int main(void)
{
	LOG_INF("02_GY521_MPU6050 (core0/procpu) starting");

	if (!device_is_ready(dev_i2c0)) {
		LOG_ERR("I2C0 bus not ready");
		return -1;
	}
	/* MPU6050 presence (0x68 vs 0x69) is resolved inside
	 * mpu6050_thread_fn itself, not here - see mpu6050_select().
	 */
	if (!device_is_ready(dev_ipm)) {
		LOG_ERR("IPM device not ready - core1 will never receive sensor data");
	}

	/* Threads above are already running (K_THREAD_DEFINE with 0 delay);
	 * main() has nothing left to do.
	 */
	return 0;
}
