/*
 * main.c - 01_AHT20_BMP280_MultiSensor (core1 / appcpu image)
 *
 * This core does exactly one job: receive sensor snapshots from core0
 * over IPM and draw them on the SSD1306. It never touches I2C0, AHT20,
 * or BMP280 - see src/main.c (core0) and the design doc for why the
 * two cores each own a different physical I2C controller.
 *
 * Event flow: the ESP32 IPM driver invokes `ipm_callback()` in
 * interrupt context whenever core0 sends a message. ISR context must
 * stay short and must not do blocking I2C transfers, so the callback
 * only copies the payload into a message queue (k_msgq_put with
 * K_NO_WAIT is ISR-safe) and returns immediately. The actual OLED
 * redraw happens in `display_thread_fn()`, which blocks on
 * `k_msgq_get()` and only wakes when a new sample has actually
 * arrived - the event-driven half of this lab's design.
 *
 * core0 does NOT send on a fixed short period any more - it pushes
 * immediately when AHT20 temperature moves >=1C or BMP280 pressure
 * moves >=10 hPa since the last value it sent, plus a 1 s heartbeat so
 * this side has something to declare "alive" even during quiet
 * periods. AHT20 humidity and BMP280 temperature are "passengers" in
 * every payload - they're always included and always drawn, but they
 * never trigger a push by themselves (see core0's main.c). This side
 * only cares about elapsed time since the last arrival: if nothing
 * shows up in `sensor_msgq` for 2 s, the link is considered down.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "ipm_protocol.h"
#include "ssd1306_display.h"

LOG_MODULE_REGISTER(app_appcpu, LOG_LEVEL_INF);

static const struct device *dev_i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
static const struct device *dev_ipm  = DEVICE_DT_GET(DT_NODELABEL(ipm0));

/* Depth 4: core0 only ever sends one relevant message every 500 ms, so
 * this just needs enough slack to absorb a brief scheduling delay on
 * the display thread - it is not meant to buffer a long backlog.
 */
K_MSGQ_DEFINE(sensor_msgq, sizeof(struct ipm_sensor_payload), 4, 4);

/* Runs in IPM ISR context - keep it minimal. */
static void ipm_callback(const struct device *ipmdev, void *user_data,
			  uint32_t id, volatile void *data)
{
	ARG_UNUSED(ipmdev);
	ARG_UNUSED(user_data);

	if (id != IPM_SENSOR_CHANNEL) {
		return; /* not our channel - ignore */
	}

	struct ipm_sensor_payload payload;

	memcpy(&payload, (const void *)data, sizeof(payload));

	if (k_msgq_put(&sensor_msgq, &payload, K_NO_WAIT) != 0) {
		/* Queue momentarily full (display thread fell behind) -
		 * drop the oldest entry and keep the newest one, since
		 * only the latest snapshot is ever useful for a live
		 * display.
		 */
		struct ipm_sensor_payload discard;

		k_msgq_get(&sensor_msgq, &discard, K_NO_WAIT);
		k_msgq_put(&sensor_msgq, &payload, K_NO_WAIT);
	}
}

static void format_fixed1(float value, char *buf, size_t buf_len)
{
	bool negative = value < 0.0f;
	float abs_value = negative ? -value : value;
	int scaled = (int)(abs_value * 10.0f + 0.5f);

	snprintf(buf, buf_len, "%s%d.%d", negative ? "-" : "", scaled / 10, scaled % 10);
}

static void render_payload(const struct ipm_sensor_payload *p, bool stale)
{
	char line[22];
	char num[8];

	ssd1306_clear();

	/* Link health goes first/top - it qualifies how much to trust
	 * everything else on the screen.
	 */
	ssd1306_draw_text(0, 0, stale ? "LINK: FAIL" : "LINK: OK");

	if (p->aht20_valid) {
		format_fixed1(p->aht20_temp_c, num, sizeof(num));
		snprintf(line, sizeof(line), "AHT20 T:%sC", num);
		ssd1306_draw_text(2, 0, line);

		format_fixed1(p->aht20_humidity_pct, num, sizeof(num));
		snprintf(line, sizeof(line), "AHT20 H:%s%%", num);
		ssd1306_draw_text(3, 0, line);
	} else {
		ssd1306_draw_text(2, 0, "AHT20 ---");
	}

	if (p->bmp280_valid) {
		format_fixed1(p->bmp280_temp_c, num, sizeof(num));
		snprintf(line, sizeof(line), "BMP280 T:%sC", num);
		ssd1306_draw_text(5, 0, line);

		snprintf(line, sizeof(line), "BMP280 P:%d hPa",
			 (int)(p->bmp280_pressure_hpa + 0.5f));
		ssd1306_draw_text(6, 0, line);
	} else {
		ssd1306_draw_text(5, 0, "BMP280 ---");
	}

	ssd1306_flush();
}

static void display_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int ret = ssd1306_display_init(dev_i2c1);

	if (ret) {
		LOG_ERR("SSD1306 init failed: %d - display thread exiting", ret);
		return;
	}

	ssd1306_draw_text(2, 0, "WAITING FOR");
	ssd1306_draw_text(3, 0, "CORE0...");
	ssd1306_flush();

	bool have_data = false;
	struct ipm_sensor_payload last_payload = { 0 };

	while (1) {
		struct ipm_sensor_payload payload;
		/* core0's heartbeat alone guarantees a message at least
		 * every 1 s even with no threshold crossings, so 2000 ms
		 * of total silence means something upstream stalled
		 * (sensor threads stuck, IPM channel jammed, core0
		 * crashed, etc) rather than just "values held steady".
		 */
		int ret2 = k_msgq_get(&sensor_msgq, &payload, K_MSEC(2000));

		if (ret2 == 0) {
			last_payload = payload;
			have_data = true;
			render_payload(&last_payload, false);
		} else if (have_data) {
			render_payload(&last_payload, true);
		}
		/* else: never received anything yet - keep showing the
		 * "WAITING FOR CORE0..." screen drawn above.
		 */
	}
}

K_THREAD_DEFINE(display_tid, 2048, display_thread_fn, NULL, NULL, NULL, 6, 0, 0);

int main(void)
{
	LOG_INF("01_AHT20_BMP280_MultiSensor (core1/appcpu) starting");

	if (!device_is_ready(dev_i2c1)) {
		LOG_ERR("I2C1 bus not ready");
		return -1;
	}
	if (!device_is_ready(dev_ipm)) {
		LOG_ERR("IPM device not ready - cannot receive sensor data from core0");
		return -1;
	}

	ipm_register_callback(dev_ipm, ipm_callback, NULL);
	ipm_set_enabled(dev_ipm, 1);

	/* display_thread_fn is already running (K_THREAD_DEFINE, 0 delay). */
	return 0;
}
