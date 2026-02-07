/*
 * safety.h — Failsafe and watchdog logic
 */

#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>
#include "joystick.h"
#include "mavlink_packer.h"

void     safety_init(void);

/*  Check joystick health.  If a failsafe condition is active,
 *  *ctrl is overwritten with neutral (all zeros).               */
void     safety_update(const joystick_state_t *js,
                       manual_control_t *ctrl);

bool     safety_is_failsafe(void);

/*  Monotonic millisecond clock (CLOCK_MONOTONIC).  */
uint64_t time_ms(void);

#endif /* SAFETY_H */
