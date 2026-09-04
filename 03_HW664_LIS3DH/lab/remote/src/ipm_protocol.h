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

/* IPM channel used for procpu -> appcpu sensor forwarding. Same
 * channel number as Lab 01/02 - the labs are never flashed onto the
 * same board at once, so there's no real conflict.
 */
#define IPM_SENSOR_CHANNEL 2

/* 3-axis acceleration (g). No temperature channel this time - the
 * actual chip on this board is LIS3DH (see design doc), and Zephyr's
 * lis2dh driver doesn't expose a temperature channel for it.
 */
struct ipm_sensor_payload {
	float accel_x_g;
	float accel_y_g;
	float accel_z_g;
	uint8_t valid;
};

#endif /* IPM_PROTOCOL_H_ */
