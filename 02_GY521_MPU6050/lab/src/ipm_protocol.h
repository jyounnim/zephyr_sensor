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
 * project's IPM convention (see Lab 01), channels 0/1 are reserved by
 * the platform and 2/3 are free for application use. This lab claims
 * channel 2, same as Lab 01 - the two labs are never built/flashed
 * onto the same device at the same time, so there is no real conflict.
 */
#define IPM_SENSOR_CHANNEL 2

/* One full MPU6050 reading: 3-axis acceleration (g), 3-axis angular
 * rate (deg/s), and die temperature (C). All seven floats + one byte
 * comfortably fit inside the ESP32 IPM driver's 64-byte per-message
 * limit (this struct is 29 bytes).
 */
struct ipm_sensor_payload {
	float accel_x_g;
	float accel_y_g;
	float accel_z_g;

	float gyro_x_dps;
	float gyro_y_dps;
	float gyro_z_dps;

	float temp_c;

	uint8_t valid;
};

#endif /* IPM_PROTOCOL_H_ */
