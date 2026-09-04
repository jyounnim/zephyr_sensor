/*
 * main.c - 03_HW664_LIS3DH (core1 / appcpu image)
 *
 * CHIP IDENTITY CORRECTION (2026-09-04): the actual chip on this board
 * is an ST LIS3DH (WHO_AM_I=0x33 confirmed on real hardware), not a
 * genuine LIS3DSH (which would read 0x3F) despite the module's
 * silkscreen/listing. This core doesn't care either way - it only
 * ever sees the 3 accel values core0 forwards over IPM.
 *
 * Same pattern as Lab 01/02's core1: receive sensor snapshots from
 * core0 over IPM and draw them on the SSD1306. This core never touches
 * I2C0 or the accelerometer directly.
 *
 * Event flow: the ESP32 IPM driver invokes `ipm_callback()` in
 * interrupt context whenever core0 sends a message. ISR context must
 * stay short, so the callback only copies the payload into a message
 * queue (k_msgq_put with K_NO_WAIT is ISR-safe) and returns
 * immediately. The actual OLED redraw happens in `display_thread_fn()`,
 * which blocks on `k_msgq_get()` and only wakes when a new sample has
 * actually arrived.
 *
 * core0 pushes immediately when the accel vector magnitude moves past
 * a threshold since the last value it sent, plus a 1s heartbeat so
 * this side has something to declare "alive" during quiet periods.
 * This side only cares about elapsed time since the last arrival: if
 * nothing shows up in `sensor_msgq` for 2 s, the link is considered
 * down.
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
		struct ipm_sensor_payload discard;

		k_msgq_get(&sensor_msgq, &discard, K_NO_WAIT);
		k_msgq_put(&sensor_msgq, &payload, K_NO_WAIT);
	}
}

static void format_fixed(float value, int decimals, char *buf, size_t buf_len)
{
	bool negative = value < 0.0f;
	float abs_value = negative ? -value : value;
	int scale = (decimals == 2) ? 100 : 10;
	int scaled = (int)(abs_value * scale + 0.5f);

	if (decimals == 2) {
		snprintf(buf, buf_len, "%s%d.%02d", negative ? "-" : "",
			  scaled / scale, scaled % scale);
	} else {
		snprintf(buf, buf_len, "%s%d.%d", negative ? "-" : "",
			  scaled / scale, scaled % scale);
	}
}

static void render_payload(const struct ipm_sensor_payload *p, bool stale)
{
	char line[22];
	char num[10];

	ssd1306_clear();

	/* Link health goes first/top - it qualifies how much to trust
	 * everything else on the screen.
	 */
	ssd1306_draw_text(0, 0, stale ? "LINK: FAIL" : "LINK: OK");

	if (p->valid) {
		format_fixed(p->accel_x_g, 2, num, sizeof(num));
		snprintf(line, sizeof(line), "AX:%s", num);
		ssd1306_draw_text(2, 0, line);

		format_fixed(p->accel_y_g, 2, num, sizeof(num));
		snprintf(line, sizeof(line), "AY:%s", num);
		ssd1306_draw_text(3, 0, line);

		format_fixed(p->accel_z_g, 2, num, sizeof(num));
		snprintf(line, sizeof(line), "AZ:%s", num);
		ssd1306_draw_text(4, 0, line);
	} else {
		ssd1306_draw_text(3, 0, "LIS3DH ---");
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

	ssd1306_draw_text(3, 0, "WAITING FOR");
	ssd1306_draw_text(4, 0, "CORE0...");
	ssd1306_flush();

	bool have_data = false;
	struct ipm_sensor_payload last_payload = { 0 };

	while (1) {
		struct ipm_sensor_payload payload;
		int ret2 = k_msgq_get(&sensor_msgq, &payload, K_MSEC(2000));

		if (ret2 == 0) {
			last_payload = payload;
			have_data = true;
			render_payload(&last_payload, false);
		} else if (have_data) {
			render_payload(&last_payload, true);
		}
	}
}

K_THREAD_DEFINE(display_tid, 2048, display_thread_fn, NULL, NULL, NULL, 6, 0, 0);

int main(void)
{
	LOG_INF("03_HW664_LIS3DH (core1/appcpu) starting");

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
