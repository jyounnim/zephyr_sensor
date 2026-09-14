/*
 * sensor_hub.c  (core0 / procpu only)
 */

#include "sensor_hub.h"
#include <math.h>
#include <stdbool.h>

static float last_baseline_mag_g;
static bool  have_baseline;

bool sensor_hub_accel_exceeds_threshold(float ax, float ay, float az, float threshold)
{
	float mag = sqrtf(ax * ax + ay * ay + az * az);
	bool triggered = (!have_baseline) || (fabsf(mag - last_baseline_mag_g) >= threshold);

	if (triggered) {
		last_baseline_mag_g = mag;
		have_baseline = true;
	}

	return triggered;
}
