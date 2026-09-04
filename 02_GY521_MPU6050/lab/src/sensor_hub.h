/*
 * sensor_hub.h  (core0 / procpu only)
 *
 * Holds the latest full MPU6050 reading. Unlike Lab 01 (two sensors on
 * one bus), only one thread here ever touches I2C0, so there is no
 * bus-contention concern and no need for a bus-serializing mutex -
 * g_data_lock below exists purely to make the snapshot struct safe to
 * read from the heartbeat thread while the sensor thread is writing
 * to it.
 */

#ifndef SENSOR_HUB_H_
#define SENSOR_HUB_H_

#include <zephyr/kernel.h>
#include <stdbool.h>

struct sensor_snapshot {
	float accel_x_g;
	float accel_y_g;
	float accel_z_g;

	float gyro_x_dps;
	float gyro_y_dps;
	float gyro_z_dps;

	float temp_c;

	bool valid;
	int  fail_count;
};

extern struct k_mutex g_data_lock;
extern struct sensor_snapshot g_snapshot;

/* Called only from mpu6050_thread, every ~100 ms. Updates the full
 * reading (all 6 axes + temperature) as one unit.
 */
void sensor_hub_update(float ax, float ay, float az,
			float gx, float gy, float gz,
			float temp_c, bool ok);

/* Called from the heartbeat thread (and indirectly by main.c's push
 * helper) to get a consistent point-in-time copy for building the IPM
 * payload.
 */
void sensor_hub_get_snapshot(struct sensor_snapshot *out);

/* Compares the current accel vector magnitude (sqrt(ax^2+ay^2+az^2),
 * in g) against the magnitude that was last actually sent to core1.
 * If they differ by at least `threshold`, records the new magnitude
 * as the baseline and returns true - the caller should push an IPM
 * message right away.
 */
bool sensor_hub_accel_exceeds_threshold(float ax, float ay, float az, float threshold);

/* Same idea for the gyro vector magnitude, in deg/s. */
bool sensor_hub_gyro_exceeds_threshold(float gx, float gy, float gz, float threshold);

#endif /* SENSOR_HUB_H_ */
