/*
 * ipm_protocol.h
 *
 * Shared between the procpu (core0, src/) and appcpu (core1, remote/src/)
 * images. Sysbuild compiles these as two completely independent Zephyr
 * applications, so there is no shared build target to enforce a single
 * copy of this file - keep both copies byte-for-byte identical.
 */

#ifndef IPM_PROTOCOL_H_
#define IPM_PROTOCOL_H_

#include <stdint.h>

/* IPM channel used for procpu -> appcpu sensor forwarding. Per this
 * project's IPM convention (see roadmap Lab 18 notes), channels 0/1
 * are reserved by the platform and 2/3 are free for application use.
 * This lab claims channel 2; channel 3 is left free for a future lab.
 */
#define IPM_SENSOR_CHANNEL 2

/* All four measured values are displayed on core1, but only two of
 * them (aht20_temp_c, bmp280_pressure_hpa) ever trigger an immediate
 * push on their own - see main.c. Humidity and BMP280 temperature are
 * "passengers": they ride along in the same payload whenever the
 * other value in their sensor triggers a send, or whenever the
 * heartbeat fires, but never trigger a send by themselves.
 *
 * Must stay within the ESP32 IPM driver's 64-byte per-message payload
 * limit (trivially true here).
 */
struct ipm_sensor_payload {
	float aht20_temp_c;
	float aht20_humidity_pct;
	uint8_t aht20_valid;

	float bmp280_temp_c;
	float bmp280_pressure_hpa;
	uint8_t bmp280_valid;
};

#endif /* IPM_PROTOCOL_H_ */
