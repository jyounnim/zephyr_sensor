/*
 * sensor_hub.h  (core0 / procpu only)
 *
 * Trimmed down for Lab 04 (Power Mode) to the one piece of shared
 * logic this lab actually needs: deciding whether the accelerometer
 * has moved enough (>= threshold g) since the last time that decision
 * fired "yes". main.c reuses this same baseline both to fire an
 * immediate Normal-mode alert and to detect the Sleep/Ultra-Sleep
 * wake condition - same math, two different reactions.
 */

#ifndef SENSOR_HUB_H_
#define SENSOR_HUB_H_

#include <stdbool.h>

/* Compares the current accel vector magnitude (sqrt(ax^2+ay^2+az^2),
 * in g) against the magnitude last recorded as a baseline. If they
 * differ by at least `threshold`, records the new magnitude as the
 * baseline and returns true.
 */
bool sensor_hub_accel_exceeds_threshold(float ax, float ay, float az, float threshold);

#endif /* SENSOR_HUB_H_ */
