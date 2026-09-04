/*
 * main.c - 01_AHT20_BMP280_MultiSensor (core0 / procpu image)
 *
 * Architecture (dual-core AMP via IPM, change-triggered + heartbeat)
 * ---------------------------------------------------------------------
 *   [aht20_thread]  100ms  -> reads temperature AND humidity
 *                           -> sensor_hub_update_aht20(temp, hum)
 *                           -> if |temp - last_sent_temp| >= 1.0C: push now
 *                              (humidity never triggers a push by itself -
 *                               it just rides along in that payload)
 *                           -> if temp is outside AHT20's rated range, treat
 *                              the sensor as stuck and self-heal - see
 *                              "AHT20 self-heal" section below.
 *
 *   [bmp280_thread] 100ms  -> reads temperature AND pressure
 *                           -> sensor_hub_update_bmp280(temp, press)
 *                           -> if |press - last_sent_press| >= 10 hPa: push now
 *                              (BMP280 temperature never triggers a push by
 *                               itself either - same "passenger" treatment)
 *
 *   [heartbeat_thread] 1000ms -> push now, unconditionally
 *
 * All three "push now" paths funnel through push_snapshot_to_core1(),
 * which always sends the FULL current snapshot (all four values), and
 * is itself serialized by g_send_lock so concurrent triggers never
 * call ipm_send() at the same time.
 *
 * Practical effect of the "passenger" design: if AHT20 humidity alone
 * swings wildly but temperature stays flat, core1 won't see it until
 * either the next BMP280 pressure threshold trigger or the next
 * heartbeat (<=1s later) - never longer than that, but not instant
 * either. Documented in the troubleshooting doc.
 *
 * AHT20 self-heal
 * ---------------
 * Confirmed on real hardware (2026-09-04): this particular AHT20 unit
 * can get stuck returning a fixed, CRC-VALID-but-physically-impossible
 * reading (~145C / ~50% humidity - the humidity value matches exactly
 * half of the ADC's full 20-bit range, the classic signature of an
 * unconverted/reset ADC register rather than a real measurement). A
 * passing CRC only proves the bytes weren't corrupted in transit, not
 * that they represent a real conversion - so Zephyr's dht20.c driver
 * (which only checks CRC and the busy bit, not physical plausibility)
 * happily returns this stuck value forever. The only thing that was
 * confirmed to clear it was fully removing and reapplying the sensor's
 * own VCC - a Zephyr MCU reset alone does NOT reset the sensor chip
 * itself, since AHT20 stays powered across an MCU reset.
 *
 * Since we can't rely on someone physically power-cycling the sensor
 * in the field, aht20_thread_fn checks every reading against AHT20's
 * datasheet-rated range (-40C..85C). Anything outside that is treated
 * as "stuck", and the thread issues the AHT20's documented soft-reset
 * command (0xBA, no payload, ~20ms to complete per datasheet) directly
 * over I2C - bypassing the Zephyr sensor API entirely, since it has no
 * public hook for this. This mimics a real power-on reset closely
 * enough in practice to clear the stuck state without any hardware
 * change.
 *
 * core0 owns I2C0 (AHT20 + BMP280). core1 owns I2C1 (SSD1306) - see
 * the design doc for why the two cores never share a peripheral.
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

/* Devicetree handles - see boards/esp32s3_devkitc_esp32s3_procpu.overlay */
static const struct device *dev_aht20  = DEVICE_DT_GET(DT_NODELABEL(aht20));
static const struct device *dev_bmp280 = DEVICE_DT_GET(DT_NODELABEL(bmp280));
static const struct device *dev_i2c0   = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct device *dev_ipm    = DEVICE_DT_GET(DT_NODELABEL(ipm0));

#define SENSOR_PERIOD_MS       100
#define HEARTBEAT_PERIOD_MS    1000 /* must stay well under core1's 2 s link timeout */
#define TEMP_THRESHOLD_C       1.0f
#define PRESSURE_THRESHOLD_HPA 10.0f

/* AHT20 datasheet operating range: -40C to +85C. Anything outside this
 * cannot be a real reading and is treated as "sensor stuck" - see the
 * "AHT20 self-heal" comment above for why this is necessary and safe.
 */
#define AHT20_PLAUSIBLE_MIN_C -40.0f
#define AHT20_PLAUSIBLE_MAX_C  85.0f

/* AHT20/AHT10 documented soft-reset command - no address register, no
 * payload, single command byte. Not exposed by Zephyr's dht20.c driver
 * API, so this talks to the chip directly over I2C0. Must only be
 * called while holding g_i2c0_bus_lock.
 */
#define AHT20_I2C_ADDR       0x38
#define AHT20_CMD_SOFTRESET  0xBA
#define AHT20_SOFTRESET_SETTLE_MS 30 /* datasheet: completes within 20ms */

static int aht20_soft_reset(const struct device *i2c_dev)
{
	uint8_t cmd = AHT20_CMD_SOFTRESET;
	int ret = i2c_write(i2c_dev, &cmd, 1, AHT20_I2C_ADDR);

	if (ret == 0) {
		k_msleep(AHT20_SOFTRESET_SETTLE_MS);
	}

	return ret;
}

/* Serializes the act of pushing an IPM message - up to three threads
 * (aht20, bmp280, heartbeat) can decide to push at nearly the same
 * instant, and ipm_send() should not be called concurrently.
 */
K_MUTEX_DEFINE(g_send_lock);

static void push_snapshot_to_core1(const char *reason)
{
	struct sensor_snapshot snap;

	sensor_hub_get_snapshot(&snap);

	/* Always the FULL snapshot - humidity and BMP280 temperature
	 * ride along here even though they didn't cause this push.
	 */
	struct ipm_sensor_payload payload = {
		.aht20_temp_c = snap.aht20_temp_c,
		.aht20_humidity_pct = snap.aht20_humidity_pct,
		.aht20_valid = snap.aht20_valid,
		.bmp280_temp_c = snap.bmp280_temp_c,
		.bmp280_pressure_hpa = snap.bmp280_pressure_hpa,
		.bmp280_valid = snap.bmp280_valid,
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

/* ---------------------------------------------------------------------
 * AHT20 thread - temperature + humidity, every 100 ms.
 * Only temperature (>=1C change) triggers an immediate push.
 * ------------------------------------------------------------------- */
static void aht20_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		struct sensor_value temp, hum;
		int ret;

		/* Hold the bus lock across the whole fetch+get sequence,
		 * not just the I2C calls inside it - the trigger command,
		 * the ~80ms conversion wait, and the follow-up read must
		 * all complete before BMP280's thread is allowed to touch
		 * I2C0. See sensor_hub.h for why.
		 */
		k_mutex_lock(&g_i2c0_bus_lock, K_FOREVER);

		/* sensor_sample_fetch() drives the AHT20's internal 0xAC
		 * trigger + ~80ms conversion wait; note this alone eats
		 * most of the 100ms loop budget - see the troubleshooting
		 * doc for what that means for the achievable read rate.
		 */
		ret = sensor_sample_fetch(dev_aht20);

		if (ret == 0) {
			ret = sensor_channel_get(dev_aht20, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			ret |= sensor_channel_get(dev_aht20, SENSOR_CHAN_HUMIDITY, &hum);
		}

		k_mutex_unlock(&g_i2c0_bus_lock);

		if (ret == 0) {
			float temp_c = sensor_value_to_double(&temp);
			float hum_pct = sensor_value_to_double(&hum);

			if (temp_c < AHT20_PLAUSIBLE_MIN_C || temp_c > AHT20_PLAUSIBLE_MAX_C) {
				/* Physically impossible reading with a valid
				 * CRC - the chip is stuck (see file header
				 * comment). Soft-reset it directly and treat
				 * this cycle as a failed read; the next
				 * cycle should come back clean.
				 */
				int temp_tenths = (int)(temp_c * 10.0f);

				LOG_WRN("AHT20 implausible temp %d.%01dC - soft-resetting",
					temp_tenths / 10, temp_tenths % 10 < 0
						? -(temp_tenths % 10) : temp_tenths % 10);

				k_mutex_lock(&g_i2c0_bus_lock, K_FOREVER);
				int reset_ret = aht20_soft_reset(dev_i2c0);
				k_mutex_unlock(&g_i2c0_bus_lock);

				if (reset_ret) {
					LOG_WRN("AHT20 soft-reset command failed: %d", reset_ret);
				}

				sensor_hub_update_aht20(0.0f, 0.0f, false);
			} else {
				sensor_hub_update_aht20(temp_c, hum_pct, true);

				/* Humidity intentionally has no threshold check of
				 * its own - see file header comment.
				 */
				if (sensor_hub_temp_exceeds_threshold(temp_c, TEMP_THRESHOLD_C)) {
					push_snapshot_to_core1("aht20-temp-threshold");
				}
			}
		} else {
			LOG_WRN("AHT20 read failed: %d", ret);
			sensor_hub_update_aht20(0.0f, 0.0f, false);
		}

		k_sleep(K_MSEC(SENSOR_PERIOD_MS));
	}
}

/* ---------------------------------------------------------------------
 * BMP280 thread - temperature + pressure, every 100 ms.
 * Only pressure (>=10 hPa change) triggers an immediate push.
 * ------------------------------------------------------------------- */
static void bmp280_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		struct sensor_value temp, press;
		int ret;

		/* Same bus lock as the AHT20 thread - see sensor_hub.h.
		 * Practical consequence: since AHT20's fetch holds this
		 * lock for ~80ms out of every 100ms cycle, this thread's
		 * actual read cadence will often be pushed later than a
		 * clean 100ms, not exactly on time - correctness (never
		 * interleaving I2C0 transactions) is prioritized over
		 * strict timing here.
		 */
		k_mutex_lock(&g_i2c0_bus_lock, K_FOREVER);

		ret = sensor_sample_fetch(dev_bmp280);

		if (ret == 0) {
			ret = sensor_channel_get(dev_bmp280, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			ret |= sensor_channel_get(dev_bmp280, SENSOR_CHAN_PRESS, &press);
		}

		k_mutex_unlock(&g_i2c0_bus_lock);

		if (ret == 0) {
			float temp_c = sensor_value_to_double(&temp);
			/* Zephyr's bme280 driver reports SENSOR_CHAN_PRESS
			 * in kPa; this series uses hPa, so convert once here.
			 */
			float pressure_hpa = sensor_value_to_double(&press) * 10.0f;

			sensor_hub_update_bmp280(temp_c, pressure_hpa, true);

			/* BMP280 temperature intentionally has no threshold
			 * check of its own - see file header comment.
			 */
			if (sensor_hub_pressure_exceeds_threshold(pressure_hpa,
								   PRESSURE_THRESHOLD_HPA)) {
				push_snapshot_to_core1("bmp280-pressure-threshold");
			}
		} else {
			LOG_WRN("BMP280 read failed: %d", ret);
			sensor_hub_update_bmp280(0.0f, 0.0f, false);
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
K_THREAD_DEFINE(aht20_tid, 1024, aht20_thread_fn, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(bmp280_tid, 1024, bmp280_thread_fn, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(heartbeat_tid, 1024, heartbeat_thread_fn, NULL, NULL, NULL, 6, 0, 0);

int main(void)
{
	LOG_INF("01_AHT20_BMP280_MultiSensor (core0/procpu) starting");

	if (!device_is_ready(dev_i2c0)) {
		LOG_ERR("I2C0 bus not ready");
		return -1;
	}
	if (!device_is_ready(dev_aht20)) {
		LOG_ERR("AHT20 device not ready - check devicetree overlay / wiring");
	}
	if (!device_is_ready(dev_bmp280)) {
		LOG_ERR("BMP280 device not ready - check devicetree overlay / wiring");
	}
	if (!device_is_ready(dev_ipm)) {
		LOG_ERR("IPM device not ready - core1 will never receive sensor data");
	}

	/* Threads above are already running (K_THREAD_DEFINE with 0 delay);
	 * main() has nothing left to do.
	 */
	return 0;
}
