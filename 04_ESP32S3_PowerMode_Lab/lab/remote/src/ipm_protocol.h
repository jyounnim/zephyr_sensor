#ifndef IPM_PROTOCOL_H_
#define IPM_PROTOCOL_H_

/*
 * ipm_protocol.h - shared core0(procpu) <-> core1(appcpu) IPM message
 * format for Lab 04 (Power Mode). This exact file must be kept
 * identical in both lab/src/ and lab/remote/src/ - there is no shared
 * include path between the two sysbuild images, so it is duplicated
 * on purpose. If you change one copy, change the other the same way.
 */

#include <stdint.h>

/* Devicetree label of the soft-IPM mailbox node shared by both images
 * (see &ipm0 in both overlays). */
#define IPM_CHANNEL_NAME "IPM_0"

typedef enum {
	/* core0 -> core1, every 1000 ms: rolling average of the last 10
	 * (100 ms) accel samples. Only sent while core1's display is
	 * active (Normal mode) - no point pushing data nobody can see. */
	IPM_CMD_ACCEL_AVG = 1,

	/* core0 -> core1, sent the instant |delta| >= 0.15 g is detected
	 * while already in Normal mode. Does not by itself change mode -
	 * it is just an out-of-cycle "look now" push. */
	IPM_CMD_ACCEL_ALERT,

	/* core0 -> core1: power the OLED off (Sleep mode). core1 must
	 * stop expecting periodic data until DISPLAY_WAKE arrives. */
	IPM_CMD_DISPLAY_SLEEP,

	/* core0 -> core1: re-power the OLED and re-initialize it from
	 * scratch. The panel was fully power-gated, so its internal
	 * controller state (and our GDDRAM mirror) must be rebuilt, not
	 * just resumed. */
	IPM_CMD_DISPLAY_WAKE,
} ipm_cmd_t;

typedef struct {
	ipm_cmd_t cmd;
	float x;
	float y;
	float z;
} ipm_msg_t;

#endif /* IPM_PROTOCOL_H_ */
