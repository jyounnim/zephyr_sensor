/*
 * sensor_hub.c  (core0 / procpu only)
 */

#include "sensor_hub.h"
#include <math.h>

K_MUTEX_DEFINE(g_data_lock);
K_MUTEX_DEFINE(g_i2c0_bus_lock);

struct sensor_snapshot g_snapshot;

/* "Last actually sent to core1" baselines for the two thresholded
 * channels only. Guarded by g_data_lock as well.
 */
static float last_sent_aht20_temp_c;
static bool  have_sent_aht20_temp;
static float last_sent_bmp280_pressure_hpa;
static bool  have_sent_bmp280_pressure;

void sensor_hub_update_aht20(float temp_c, float humidity_pct, bool ok)
{
	k_mutex_lock(&g_data_lock, K_FOREVER);

	if (ok) {
		g_snapshot.aht20_temp_c = temp_c;
		g_snapshot.aht20_humidity_pct = humidity_pct;
		g_snapshot.aht20_valid = true;
		g_snapshot.aht20_fail_count = 0;
	} else {
		g_snapshot.aht20_fail_count++;
		/* Keep the last good reading around; only drop `valid`
		 * after a few consecutive failures so a single noisy
		 * transaction doesn't blank core1's display.
		 */
		if (g_snapshot.aht20_fail_count >= 3) {
			g_snapshot.aht20_valid = false;
		}
	}

	k_mutex_unlock(&g_data_lock);
}

void sensor_hub_update_bmp280(float temp_c, float pressure_hpa, bool ok)
{
	k_mutex_lock(&g_data_lock, K_FOREVER);

	if (ok) {
		g_snapshot.bmp280_temp_c = temp_c;
		g_snapshot.bmp280_pressure_hpa = pressure_hpa;
		g_snapshot.bmp280_valid = true;
		g_snapshot.bmp280_fail_count = 0;
	} else {
		g_snapshot.bmp280_fail_count++;
		if (g_snapshot.bmp280_fail_count >= 3) {
			g_snapshot.bmp280_valid = false;
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

bool sensor_hub_temp_exceeds_threshold(float new_temp_c, float threshold)
{
	bool trigger;

	k_mutex_lock(&g_data_lock, K_FOREVER);

	trigger = (!have_sent_aht20_temp) ||
		  (fabsf(new_temp_c - last_sent_aht20_temp_c) >= threshold);
	if (trigger) {
		last_sent_aht20_temp_c = new_temp_c;
		have_sent_aht20_temp = true;
	}

	k_mutex_unlock(&g_data_lock);

	return trigger;
}

bool sensor_hub_pressure_exceeds_threshold(float new_pressure_hpa, float threshold)
{
	bool trigger;

	k_mutex_lock(&g_data_lock, K_FOREVER);

	trigger = (!have_sent_bmp280_pressure) ||
		  (fabsf(new_pressure_hpa - last_sent_bmp280_pressure_hpa) >= threshold);
	if (trigger) {
		last_sent_bmp280_pressure_hpa = new_pressure_hpa;
		have_sent_bmp280_pressure = true;
	}

	k_mutex_unlock(&g_data_lock);

	return trigger;
}
