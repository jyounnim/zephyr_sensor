/*
 * main.c - 03_HW664_LIS3DH (core0 / procpu image)
 *
 * CHIP IDENTITY CORRECTION (2026-09-04): confirmed via WHO_AM_I=0x33
 * on real hardware that this HW-664 board actually carries an ST
 * LIS3DH (WHO_AM_I 0x3F would have meant a genuine LIS3DSH). Zephyr
 * officially supports LIS3DH via its LIS2DH driver family
 * (compatible "st,lis3dh"), so this revision uses the standard
 * sensor_sample_fetch()/sensor_channel_get() API instead of the
 * hand-rolled raw-I2C driver an earlier revision wrote when this was
 * still believed to be an unsupported LIS3DSH.
 *
 * Architecture (dual-core AMP via IPM, change-triggered + heartbeat)
 * ---------------------------------------------------------------------
 *   [lis3dh_thread] 100ms -> reads accel XYZ
 *                          -> sensor_hub_update(...)
 *                          -> if accel vector magnitude moves >=0.15g
 *                             since the last value actually sent: push now
 *
 *   [heartbeat_thread] 1000ms -> push now, unconditionally
 *
 * Address auto-detect (0x18 vs 0x19)
 * -----------------------------------
 * Confirmed on this unit: SDO/SA0 unconnected/HIGH -> 0x19, LOW ->
 * 0x18. Rather than hardcoding one, the overlay declares both as
 * separate devicetree nodes; the lis2dh driver's init() reads
 * WHO_AM_I and fails (device_is_ready() == false) for whichever
 * address has no real hardware behind it. lis3dh_select(), called
 * from lis3dh_thread_fn's own startup (not from main() - same
 * K_THREAD_DEFINE-vs-main() ordering reasoning as Lab 02), picks
 * whichever candidate actually came up ready.
 *
 * core0 owns I2C0 (LIS3DH). core1 owns I2C1 (SSD1306) - see the
 * design doc for why the two cores never share a peripheral.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/logging/log.h>

#include "sensor_hub.h"
#include "ipm_protocol.h"

LOG_MODULE_REGISTER(app_procpu, LOG_LEVEL_INF);

/* Devicetree handles - see boards/esp32s3_devkitc_esp32s3_procpu.overlay.
 * Both candidate addresses are declared there; exactly one of these
 * two will actually initialize successfully.
 */
static const struct device *dev_lis3dh_18 = DEVICE_DT_GET(DT_NODELABEL(lis3dh_18));
static const struct device *dev_lis3dh_19 = DEVICE_DT_GET(DT_NODELABEL(lis3dh_19));
static const struct device *dev_lis3dh; /* selected at boot - see lis3dh_select() */
static const struct device *dev_i2c0    = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct device *dev_ipm     = DEVICE_DT_GET(DT_NODELABEL(ipm0));

#define SENSOR_PERIOD_MS      100
#define HEARTBEAT_PERIOD_MS   1000 /* must stay well under core1's 2 s link timeout */
#define ACCEL_THRESHOLD_G     0.15f

#define STANDARD_GRAVITY_MS2  9.80665f

K_MUTEX_DEFINE(g_send_lock);

static void push_snapshot_to_core1(const char *reason)
{
	struct sensor_snapshot snap;

	sensor_hub_get_snapshot(&snap);

	struct ipm_sensor_payload payload = {
		.accel_x_g = snap.accel_x_g,
		.accel_y_g = snap.accel_y_g,
		.accel_z_g = snap.accel_z_g,
		.valid = snap.valid,
	};

	k_mutex_lock(&g_send_lock, K_FOREVER);
	int ret = ipm_send(dev_ipm, 0, IPM_SENSOR_CHANNEL, &payload, sizeof(payload));
	k_mutex_unlock(&g_send_lock);

	if (ret) {
		LOG_WRN("ipm_send failed (%s): %d", reason, ret);
	} else {
		LOG_DBG("ipm_send ok (%s)", reason);
	}
}

/* Picks whichever candidate address actually has hardware behind it -
 * see the file header comment for why this runs here and not in
 * main().
 */
static const struct device *lis3dh_select(void)
{
	if (device_is_ready(dev_lis3dh_18)) {
		LOG_INF("LIS3DH found at 0x18 (SDO/SA0 low)");
		return dev_lis3dh_18;
	}
	if (device_is_ready(dev_lis3dh_19)) {
		LOG_INF("LIS3DH found at 0x19 (SDO/SA0 high/unconnected)");
		return dev_lis3dh_19;
	}
	LOG_ERR("LIS3DH not found at 0x18 or 0x19 - check wiring");
	return NULL;
}

/* ---------------------------------------------------------------------
 * LIS3DH thread - accel XYZ, every 100 ms
 * ------------------------------------------------------------------- */
static void lis3dh_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	dev_lis3dh = lis3dh_select();

	if (dev_lis3dh == NULL) {
		while (1) {
			k_sleep(K_SECONDS(5));
		}
	}

	while (1) {
		struct sensor_value accel[3];
		int ret = sensor_sample_fetch(dev_lis3dh);

		if (ret == 0) {
			ret = sensor_channel_get(dev_lis3dh, SENSOR_CHAN_ACCEL_XYZ, accel);
		}

		if (ret == 0) {
			/* Driver units: accel in m/s^2 (SI), like every
			 * other Zephyr accel driver in this series.
			 */
			float ax_g = (float)(sensor_value_to_double(&accel[0]) / STANDARD_GRAVITY_MS2);
			float ay_g = (float)(sensor_value_to_double(&accel[1]) / STANDARD_GRAVITY_MS2);
			float az_g = (float)(sensor_value_to_double(&accel[2]) / STANDARD_GRAVITY_MS2);

			sensor_hub_update(ax_g, ay_g, az_g, true);

			if (sensor_hub_accel_exceeds_threshold(ax_g, ay_g, az_g,
								ACCEL_THRESHOLD_G)) {
				push_snapshot_to_core1("accel-threshold");
			}
		} else {
			LOG_WRN("LIS3DH read failed: %d", ret);
			sensor_hub_update(0.0f, 0.0f, 0.0f, false);
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

K_THREAD_DEFINE(lis3dh_tid, 1024, lis3dh_thread_fn, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(heartbeat_tid, 1024, heartbeat_thread_fn, NULL, NULL, NULL, 6, 0, 0);

int main(void)
{
	LOG_INF("03_HW664_LIS3DH (core0/procpu) starting");

	if (!device_is_ready(dev_i2c0)) {
		LOG_ERR("I2C0 bus not ready");
		return -1;
	}
	/* LIS3DH presence/address (0x18 vs 0x19) is resolved inside
	 * lis3dh_thread_fn itself, not here - see lis3dh_select().
	 */
	if (!device_is_ready(dev_ipm)) {
		LOG_ERR("IPM device not ready - core1 will never receive sensor data");
	}

	return 0;
}
