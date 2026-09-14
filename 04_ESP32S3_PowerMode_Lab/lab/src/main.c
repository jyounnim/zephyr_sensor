/*
 * main.c - Lab 04: ESP32-S3 Power Mode lab, core0 (procpu) image.
 *
 * Owns I2C0 (HW-664 / LIS3DH), the onboard Boot button, and the
 * 3-state power policy state machine. Never touches I2C1 or the
 * SSD1306 directly - core1 owns the display entirely and only ever
 * hears from this image over IPM (see ipm_protocol.h).
 *
 * State machine (Boot button cycles Normal -> Sleep -> Ultra-Sleep):
 *
 *   NORMAL:
 *     - Sample HW-664 every 100 ms.
 *     - Every 1000 ms (10 samples), push the average to core1
 *       (IPM_CMD_ACCEL_AVG).
 *     - The instant the accel vector magnitude moves >= 0.15 g since
 *       the last value actually sent, push an out-of-cycle alert
 *       (IPM_CMD_ACCEL_ALERT) right away.
 *     - Boot button press -> Sleep: tell core1 to power off the OLED.
 *
 *   SLEEP:
 *     - Still samples every 100 ms (so the wake trigger below stays
 *       responsive), but stops pushing IPM_CMD_ACCEL_AVG/ALERT - the
 *       display is off, nobody would see it anyway.
 *     - Accel delta >= 0.15 g -> back to Normal: tell core1 to power
 *       the OLED back on and re-initialize it from scratch.
 *     - Boot button press -> Ultra-Sleep.
 *
 *   ULTRA_SLEEP:
 *     - Samples only once per 1000 ms. core1 hears nothing in this
 *       state (the OLED is already off from Sleep mode, nothing to
 *       change). This is a software-only low-power *policy* (slow
 *       polling, CONFIG_PM stays off) rather than a confirmed SoC
 *       Light-Sleep demonstration - see prj.conf and
 *       doc/04_ESP32S3_PowerMode_TROUBLESHOOTING_kr.md for why.
 *     - Accel delta >= 0.15 g -> sys_reboot(): this lab treats "wake
 *       from Ultra-Sleep" as a warm reset back to Normal mode, rather
 *       than resuming in place (see prj.conf for why: sys_poweroff()
 *       is deliberately avoided here).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <math.h>
#include <errno.h>

#include "ipm_protocol.h"
#include "sensor_hub.h"

LOG_MODULE_REGISTER(app_procpu, LOG_LEVEL_INF);

/* Two candidate HW-664 (LIS3DH) devicetree nodes at 0x18/0x19 - exactly
 * one will report device_is_ready() == true on real hardware, per this
 * series' address-autodetect convention (see the overlay). */
static const struct device *const lis3dh_18 = DEVICE_DT_GET(DT_NODELABEL(lis3dh_18));
static const struct device *const lis3dh_19 = DEVICE_DT_GET(DT_NODELABEL(lis3dh_19));
static const struct device *lis3dh_dev;

static const struct gpio_dt_spec boot_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct device *ipm_dev;
static struct gpio_callback button_cb_data;

typedef enum {
	APP_MODE_NORMAL = 0,
	APP_MODE_SLEEP,
	APP_MODE_ULTRA_SLEEP,
} app_mode_t;

static app_mode_t current_mode = APP_MODE_NORMAL;

/* Set from the GPIO ISR, consumed by the main loop. GPIO callbacks run
 * in interrupt context in Zephyr - k_sleep()/ipm_send() are NOT valid
 * to call there, so the callback does nothing but raise this flag. */
static volatile bool button_event_pending;

#define ACCEL_DELTA_THRESHOLD_G   0.15f
#define SAMPLE_PERIOD_NORMAL_MS   100
#define AVERAGE_PERIOD_MS         1000
#define SAMPLES_PER_AVERAGE       (AVERAGE_PERIOD_MS / SAMPLE_PERIOD_NORMAL_MS)
#define ULTRA_SLEEP_PERIOD_MS     1000
#define STANDARD_GRAVITY_MS2      9.80665f

static void boot_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	button_event_pending = true;
}

static int lis3dh_select(void)
{
	if (device_is_ready(lis3dh_18)) {
		lis3dh_dev = lis3dh_18;
		LOG_INF("HW-664 (LIS3DH) found at I2C0 address 0x18");
		return 0;
	}
	if (device_is_ready(lis3dh_19)) {
		lis3dh_dev = lis3dh_19;
		LOG_INF("HW-664 (LIS3DH) found at I2C0 address 0x19");
		return 0;
	}

	LOG_ERR("HW-664 (LIS3DH) not found at 0x18 or 0x19 - check wiring");
	return -ENODEV;
}

/* Reads one 3-axis sample in g. Returns false (and leaves *ax/*ay/*az
 * untouched) on any sensor API error. */
static bool lis3dh_read_g(float *ax, float *ay, float *az)
{
	struct sensor_value x, y, z;

	if (sensor_sample_fetch(lis3dh_dev) < 0) {
		return false;
	}
	if (sensor_channel_get(lis3dh_dev, SENSOR_CHAN_ACCEL_X, &x) < 0 ||
	    sensor_channel_get(lis3dh_dev, SENSOR_CHAN_ACCEL_Y, &y) < 0 ||
	    sensor_channel_get(lis3dh_dev, SENSOR_CHAN_ACCEL_Z, &z) < 0) {
		return false;
	}

	/* Zephyr sensor channels report m/s^2; this lab's 0.15g threshold
	 * and on-screen units are in g. */
	*ax = (float)sensor_value_to_double(&x) / STANDARD_GRAVITY_MS2;
	*ay = (float)sensor_value_to_double(&y) / STANDARD_GRAVITY_MS2;
	*az = (float)sensor_value_to_double(&z) / STANDARD_GRAVITY_MS2;
	return true;
}

static void send_ipm(ipm_cmd_t cmd, float x, float y, float z)
{
	ipm_msg_t msg = { .cmd = cmd, .x = x, .y = y, .z = z };
	int ret = ipm_send(ipm_dev, 0, (uint32_t)cmd, &msg, sizeof(msg));

	if (ret < 0) {
		LOG_WRN("ipm_send(cmd=%d) failed: %d", cmd, ret);
	}
}

/* Runs once per main-loop iteration when button_event_pending was set.
 * This is where it's actually safe to call ipm_send()/k_sleep(). */
static void handle_button_event(void)
{
	switch (current_mode) {
	case APP_MODE_NORMAL:
		current_mode = APP_MODE_SLEEP;
		send_ipm(IPM_CMD_DISPLAY_SLEEP, 0, 0, 0);
		LOG_INF("Boot button: Normal -> Sleep");
		break;

	case APP_MODE_SLEEP:
		current_mode = APP_MODE_ULTRA_SLEEP;
		LOG_INF("Boot button: Sleep -> Ultra-Sleep");
		break;

	case APP_MODE_ULTRA_SLEEP:
		/* No third press target - Ultra-Sleep only leaves via the
		 * accel-triggered reboot described at the top of this file. */
		break;
	}
}

int main(void)
{
	int ret;

	LOG_INF("Lab 04 Power Mode (core0/procpu) starting, mode=NORMAL");

	ipm_dev = DEVICE_DT_GET(DT_NODELABEL(ipm0));
	if (!device_is_ready(ipm_dev)) {
		LOG_ERR("IPM device not ready");
		return -1;
	}

	if (lis3dh_select() != 0) {
		return -1;
	}

	if (!device_is_ready(boot_button.port)) {
		LOG_ERR("Boot button GPIO controller not ready");
		return -1;
	}
	ret = gpio_pin_configure_dt(&boot_button, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure boot button: %d", ret);
		return ret;
	}
	gpio_pin_interrupt_configure_dt(&boot_button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_data, boot_button_pressed, BIT(boot_button.pin));
	gpio_add_callback(boot_button.port, &button_cb_data);

	float sum_x = 0, sum_y = 0, sum_z = 0;
	int sample_cnt = 0;

	while (1) {
		if (button_event_pending) {
			button_event_pending = false;
			handle_button_event();
		}

		if (current_mode == APP_MODE_ULTRA_SLEEP) {
			float ax, ay, az;

			if (lis3dh_read_g(&ax, &ay, &az) &&
			    sensor_hub_accel_exceeds_threshold(ax, ay, az, ACCEL_DELTA_THRESHOLD_G)) {
				LOG_WRN("Impact detected in Ultra-Sleep (%.2f,%.2f,%.2f g) - "
					"rebooting to Normal mode", (double)ax, (double)ay, (double)az);
				k_msleep(20); /* let the log line flush before reset */
				sys_reboot(SYS_REBOOT_WARM);
				/* unreachable */
			}
			k_msleep(ULTRA_SLEEP_PERIOD_MS);
			continue;
		}

		/* Normal and Sleep both sample at 100 ms, so the 0.15g wake
		 * trigger stays responsive even with the OLED powered off. */
		float ax, ay, az;
		bool ok = lis3dh_read_g(&ax, &ay, &az);

		if (ok) {
			bool triggered = sensor_hub_accel_exceeds_threshold(
				ax, ay, az, ACCEL_DELTA_THRESHOLD_G);

			if (triggered && current_mode == APP_MODE_SLEEP) {
				current_mode = APP_MODE_NORMAL;
				send_ipm(IPM_CMD_DISPLAY_WAKE, 0, 0, 0);
				LOG_INF("Impact detected in Sleep mode - back to Normal");
			} else if (triggered && current_mode == APP_MODE_NORMAL) {
				send_ipm(IPM_CMD_ACCEL_ALERT, ax, ay, az);
			}

			sum_x += ax;
			sum_y += ay;
			sum_z += az;
			sample_cnt++;
		}

		if (sample_cnt >= SAMPLES_PER_AVERAGE) {
			if (current_mode == APP_MODE_NORMAL) {
				send_ipm(IPM_CMD_ACCEL_AVG,
					 sum_x / sample_cnt, sum_y / sample_cnt, sum_z / sample_cnt);
			}
			sum_x = sum_y = sum_z = 0;
			sample_cnt = 0;
		}

		k_msleep(SAMPLE_PERIOD_NORMAL_MS);
	}
	return 0;
}
