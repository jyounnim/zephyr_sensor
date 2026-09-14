/*
 * main.c - Lab 04: ESP32-S3 Power Mode lab, core1 (appcpu) image.
 *
 * Owns I2C1 (SSD1306 OLED) and the OLED's dedicated power-gate GPIO.
 * Never touches I2C0 or the HW-664/LIS3DH directly - it only ever
 * hears from core0 over IPM (see ipm_protocol.h).
 *
 * Event flow: the ESP32 soft-IPM driver invokes ipm_callback() in
 * interrupt context on every message from core0. ISR context must
 * stay short, so the callback only copies the payload into a message
 * queue (k_msgq_put with K_NO_WAIT is ISR-safe) and returns
 * immediately - all real work (GPIO toggling, I2C, OLED redraw)
 * happens in display_thread_fn().
 *
 * The display thread is started manually with k_thread_create() from
 * main(), AFTER the OLED power-gate GPIO and IPM device are confirmed
 * ready - not via K_THREAD_DEFINE's implicit auto-start, which would
 * race main()'s own device/GPIO setup.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#include "ipm_protocol.h"
#include "ssd1306_display.h"

LOG_MODULE_REGISTER(app_appcpu, LOG_LEVEL_INF);

static const struct device *dev_i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
static const struct device *dev_ipm  = DEVICE_DT_GET(DT_NODELABEL(ipm0));
static const struct gpio_dt_spec oled_pwr = GPIO_DT_SPEC_GET(DT_ALIAS(oled_pwr), gpios);

/* 3000 ms = 3 missed 1 s average pushes, per this lab's connection-
 * loss rule. Only enforced while the display is actually active -
 * while asleep, silence is expected, not a fault. */
#define LINK_TIMEOUT_MS 3000

K_MSGQ_DEFINE(ipm_msgq, sizeof(ipm_msg_t), 4, 4);

static K_THREAD_STACK_DEFINE(display_stack, 2048);
static struct k_thread display_thread_data;

/* Runs in IPM ISR context - keep it minimal. */
static void ipm_callback(const struct device *ipmdev, void *user_data,
			  uint32_t id, volatile void *data)
{
	ARG_UNUSED(ipmdev);
	ARG_UNUSED(user_data);
	ARG_UNUSED(id);

	ipm_msg_t msg;

	memcpy(&msg, (const void *)data, sizeof(msg));

	if (k_msgq_put(&ipm_msgq, &msg, K_NO_WAIT) != 0) {
		ipm_msg_t discard;

		/* Queue full - this only happens if core1 fell far behind;
		 * drop the oldest entry and keep the newest one. */
		k_msgq_get(&ipm_msgq, &discard, K_NO_WAIT);
		k_msgq_put(&ipm_msgq, &msg, K_NO_WAIT);
	}
}

static void format_fixed(float value, char *buf, size_t buf_len)
{
	bool negative = value < 0.0f;
	float abs_value = negative ? -value : value;
	int scaled = (int)(abs_value * 100.0f + 0.5f);

	snprintf(buf, buf_len, "%s%d.%02d", negative ? "-" : "",
		  scaled / 100, scaled % 100);
}

static void render_accel(const char *title, float x, float y, float z)
{
	char line[22];
	char num[10];

	ssd1306_clear();
	ssd1306_draw_text(0, 0, title);

	format_fixed(x, num, sizeof(num));
	snprintf(line, sizeof(line), "AX:%s", num);
	ssd1306_draw_text(2, 0, line);

	format_fixed(y, num, sizeof(num));
	snprintf(line, sizeof(line), "AY:%s", num);
	ssd1306_draw_text(3, 0, line);

	format_fixed(z, num, sizeof(num));
	snprintf(line, sizeof(line), "AZ:%s", num);
	ssd1306_draw_text(4, 0, line);

	ssd1306_flush();
}

static void show_status(const char *l1, const char *l2)
{
	ssd1306_clear();
	if (l1) {
		ssd1306_draw_text(3, 0, l1);
	}
	if (l2) {
		ssd1306_draw_text(4, 0, l2);
	}
	ssd1306_flush();
}

/* Cuts the OLED's own power rail - not a software blank. The panel
 * loses all internal state (GDDRAM, init sequence) when this happens. */
static void power_off_display(void)
{
	gpio_pin_set_dt(&oled_pwr, 0);
}

/* Re-powers the OLED and re-runs its full init sequence from scratch,
 * since power_off_display() above did not preserve any panel state. */
static int power_on_display(void)
{
	gpio_pin_set_dt(&oled_pwr, 1);

	/* Panel needs its own power-on settle time before its internal
	 * controller responds on I2C - same boot-stabilization convention
	 * as this series' other OLED labs. */
	k_msleep(100);

	int ret = ssd1306_display_init(dev_i2c1);

	if (ret) {
		LOG_ERR("SSD1306 (re-)init failed: %d", ret);
		return ret;
	}
	return 0;
}

static void display_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	bool active = true;

	if (power_on_display() == 0) {
		show_status("WAITING FOR", "CORE0...");
	}

	while (1) {
		ipm_msg_t msg;
		int ret = k_msgq_get(&ipm_msgq, &msg,
				      active ? K_MSEC(LINK_TIMEOUT_MS) : K_FOREVER);

		if (ret != 0) {
			/* Timed out waiting for the periodic average - only
			 * meaningful while the display is active and data is
			 * actually expected. */
			if (active) {
				ssd1306_draw_text(0, 0, "LINK: FAIL");
				ssd1306_flush();
			}
			continue;
		}

		switch (msg.cmd) {
		case IPM_CMD_DISPLAY_SLEEP:
			active = false;
			power_off_display();
			LOG_INF("OLED powered off (Sleep mode)");
			break;

		case IPM_CMD_DISPLAY_WAKE:
			if (power_on_display() == 0) {
				active = true;
				show_status("NORMAL MODE", NULL);
				LOG_INF("OLED powered back on (Normal mode)");
			}
			break;

		case IPM_CMD_ACCEL_AVG:
			if (active) {
				render_accel("LINK: OK", msg.x, msg.y, msg.z);
			}
			break;

		case IPM_CMD_ACCEL_ALERT:
			if (active) {
				render_accel("IMPACT!", msg.x, msg.y, msg.z);
			}
			break;

		default:
			LOG_WRN("Unknown IPM cmd %d", msg.cmd);
			break;
		}
	}
}

int main(void)
{
	LOG_INF("Lab 04 Power Mode (core1/appcpu) starting");

	if (!device_is_ready(dev_i2c1)) {
		LOG_ERR("I2C1 bus not ready");
		return -1;
	}
	if (!device_is_ready(dev_ipm)) {
		LOG_ERR("IPM device not ready - cannot receive commands from core0");
		return -1;
	}
	if (!device_is_ready(oled_pwr.port)) {
		LOG_ERR("OLED power-gate GPIO controller not ready");
		return -1;
	}

	int ret = gpio_pin_configure_dt(&oled_pwr, GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		LOG_ERR("Failed to configure OLED power-gate GPIO: %d", ret);
		return ret;
	}

	ipm_register_callback(dev_ipm, ipm_callback, NULL);
	ipm_set_enabled(dev_ipm, 1);

	k_thread_create(&display_thread_data, display_stack,
			K_THREAD_STACK_SIZEOF(display_stack),
			display_thread_fn, NULL, NULL, NULL,
			6, 0, K_NO_WAIT);

	return 0;
}
