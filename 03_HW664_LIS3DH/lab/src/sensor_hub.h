/*
 * sensor_hub.h  (core0 / procpu only)
 *
 * Holds the latest LIS3DH reading. Only one thread ever touches I2C0
 * here, so g_data_lock exists purely to make the snapshot struct safe
 * to read from the heartbeat thread while the sensor thread writes to
 * it - same pattern as Lab 02.
 */

#ifndef SENSOR_HUB_H_
#define SENSOR_HUB_H_

#include <zephyr/kernel.h>
#include <stdbool.h>

struct sensor_snapshot {
	float accel_x_g;
	float accel_y_g;
	float accel_z_g;

	bool valid;
	int  fail_count;
};

extern struct k_mutex g_data_lock;
extern struct sensor_snapshot g_snapshot;

/* Called only from the LIS3DH thread, every ~100 ms. */
void sensor_hub_update(float ax, float ay, float az, bool ok);

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

#endif /* SENSOR_HUB_H_ */
