/*
 * sensor_hub.c  (core0 / procpu only)
 */

#include "sensor_hub.h"
#include <math.h>

K_MUTEX_DEFINE(g_data_lock);

struct sensor_snapshot g_snapshot;

static float last_sent_accel_mag_g;
static bool  have_sent_accel;

void sensor_hub_update(float ax, float ay, float az, bool ok)
{
	k_mutex_lock(&g_data_lock, K_FOREVER);

	if (ok) {
		g_snapshot.accel_x_g = ax;
		g_snapshot.accel_y_g = ay;
		g_snapshot.accel_z_g = az;
		g_snapshot.valid = true;
		g_snapshot.fail_count = 0;
	} else {
		g_snapshot.fail_count++;
		if (g_snapshot.fail_count >= 3) {
			g_snapshot.valid = false;
		}
	}

	k_mutex_unlock(&g_data_lock);
}

void sensor_hub_get_snapshot(struct sensor_snapshot *out)
{
	k_mutex_lock(&g_data_lock, K_FOREVER);
	*out = g_snapshot;
	k_mutex_unlock(&g_data_lock);
}

bool sensor_hub_accel_exceeds_threshold(float ax, float ay, float az, float threshold)
{
	bool trigger;
	float mag = sqrtf(ax * ax + ay * ay + az * az);

	k_mutex_lock(&g_data_lock, K_FOREVER);

	trigger = (!have_sent_accel) || (fabsf(mag - last_sent_accel_mag_g) >= threshold);
	if (trigger) {
		last_sent_accel_mag_g = mag;
		have_sent_accel = true;
	}

	k_mutex_unlock(&g_data_lock);

	return trigger;
}
