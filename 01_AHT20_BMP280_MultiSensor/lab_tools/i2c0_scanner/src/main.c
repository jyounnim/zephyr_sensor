/*
 * main.c - I2C0 bus scanner (diagnostic tool, not part of the main lab)
 *
 * Standalone, single-core (procpu only) build - no sysbuild needed.
 * Scans every 7-bit I2C address on I2C0 (SDA=GPIO8/SCL=GPIO9, same
 * bus AHT20+BMP280 use in the main lab) and logs which ones ACK.
 *
 * Probing method: a 1-byte write with no useful payload, per this
 * series' established I2C0 convention (write-based probing was
 * chosen over read-based probing after ESP32 Zephyr I2C driver NACK
 * detection issues were found on read - see project notes / Zephyr
 * GitHub issue #45008).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c0_scanner, LOG_LEVEL_INF);

static const struct device *dev_i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

int main(void)
{
	LOG_INF("I2C0 bus scanner starting");

	if (!device_is_ready(dev_i2c0)) {
		LOG_ERR("I2C0 bus not ready");
		return -1;
	}

	/* Give any slow-to-boot sensors (AHT20's 100ms power-on wait,
	 * etc.) time to settle before probing.
	 */
	k_msleep(200);

	uint8_t dummy = 0x00;
	int found = 0;

	LOG_INF("Scanning 0x03..0x77 ...");

	/* 0x00-0x02 and 0x78-0x7F are reserved by the I2C spec
	 * (general call / high-speed mode addresses) - skip them.
	 */
	for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
		int ret = i2c_write(dev_i2c0, &dummy, 1, addr);

		if (ret == 0) {
			LOG_INF("  Found device at 0x%02X", addr);
			found++;
		}
		/* Small delay between probes - not strictly required, but
		 * keeps the bus from being hammered back-to-back and gives
		 * a NACK'd device time to release SDA cleanly before the
		 * next address is tried.
		 */
		k_msleep(5);
	}

	if (found == 0) {
		LOG_WRN("No devices found on I2C0 - check wiring/pull-ups, "
			"not just the addresses expected below.");
	} else {
		LOG_INF("Scan complete: %d device(s) found.", found);
	}

	LOG_INF("Expected in this lab: AHT20 at 0x38, BMP280 at 0x76 or 0x77.");

	return 0;
}
