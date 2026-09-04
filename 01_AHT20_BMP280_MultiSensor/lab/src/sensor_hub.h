/*
 * sensor_hub.h  (core0 / procpu only)
 *
 * Holds the latest full reading from each sensor - AHT20 gives
 * temperature + humidity, BMP280 gives temperature + pressure, all
 * four values are kept and eventually displayed on core1. Only two
 * of the four (AHT20 temperature, BMP280 pressure) have a "last sent"
 * baseline and a threshold check below; humidity and BMP280
 * temperature are carried along in whichever payload goes out next
 * (a threshold trigger from their own sensor's other channel, or the
 * heartbeat) but never trigger a send by themselves - see main.c.
 *
 * All access goes through g_data_lock.
 */

#ifndef SENSOR_HUB_H_
#define SENSOR_HUB_H_

#include <zephyr/kernel.h>
#include <stdbool.h>

struct sensor_snapshot {
	float aht20_temp_c;
	float aht20_humidity_pct;
	bool  aht20_valid;
	int   aht20_fail_count;

	float bmp280_temp_c;
	float bmp280_pressure_hpa;
	bool  bmp280_valid;
	int   bmp280_fail_count;
};

extern struct k_mutex g_data_lock;
extern struct sensor_snapshot g_snapshot;

/* Serializes ALL I2C0 traffic between the AHT20 and BMP280 threads.
 * This is deliberately separate from g_data_lock (which only protects
 * the snapshot struct): this project has an already-confirmed ESP32
 * Zephyr I2C driver reliability issue with multi-segment (repeated
 * start) transactions, and both dht20.c (AHT20) and the bme280 driver
 * rely on exactly that transaction shape internally. Wrapping each
 * sensor's ENTIRE fetch+get sequence in this lock guarantees the two
 * threads' I2C0 transactions never interleave, regardless of whether
 * the underlying platform bug would otherwise allow it.
 */
extern struct k_mutex g_i2c0_bus_lock;

/* Called only from the AHT20 thread, every ~100 ms. Updates both
 * temperature and humidity together (they come from one fetch).
 */
void sensor_hub_update_aht20(float temp_c, float humidity_pct, bool ok);

/* Called only from the BMP280 thread, every ~100 ms. Updates both
 * temperature and pressure together.
 */
void sensor_hub_update_bmp280(float temp_c, float pressure_hpa, bool ok);

/* Called from the heartbeat thread (and indirectly by main.c's push
 * helper) to get a consistent point-in-time copy for building the IPM
 * payload - includes all four values regardless of which one changed.
 */
void sensor_hub_get_snapshot(struct sensor_snapshot *out);

/* Compares `new_temp_c` (AHT20 temperature) against the value that was
 * last actually sent to core1. If they differ by at least `threshold`
 * degrees (or nothing has ever been sent yet), records the new value
 * as the baseline and returns true - the caller should push an IPM
 * message right away. Returns false otherwise. Humidity has no
 * equivalent function: it never triggers a send by itself.
 */
bool sensor_hub_temp_exceeds_threshold(float new_temp_c, float threshold);

/* Same idea for BMP280 pressure. BMP280 temperature has no equivalent
 * function: it never triggers a send by itself.
 */
bool sensor_hub_pressure_exceeds_threshold(float new_pressure_hpa, float threshold);

#endif /* SENSOR_HUB_H_ */
