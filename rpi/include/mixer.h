/*
 * mixer.h — 6-thruster mixing matrix
 *
 * Maps 4 control channels (surge, sway, heave, yaw) to 6 motor outputs.
 *
 * Thruster layout (top-down view, front = up):
 *
 *        FL ╲     ╱ FR          Horizontal thrusters (45° vectored)
 *             ╲ ╱
 *            [ROV]
 *             ╱ ╲
 *        BL ╱     ╲ BR
 *
 *         VL         VR          Vertical thrusters
 *
 * Each horizontal thruster is angled 45° from centre-line.
 * Vertical thrusters point straight up/down for heave.
 */

#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>

#define MIXER_NUM_MOTORS   6

/* Input:  surge, sway, heave, yaw — each [-1000, +1000]
 * Output: motor[0..5] — each [-1000, +1000] (mapped to PWM later)
 *
 *   motor[0] = Front-Right (FR)
 *   motor[1] = Front-Left  (FL)
 *   motor[2] = Back-Right  (BR)
 *   motor[3] = Back-Left   (BL)
 *   motor[4] = Vertical-Left  (VL)
 *   motor[5] = Vertical-Right (VR)
 */
void mixer_compute(int16_t surge, int16_t sway, int16_t heave, int16_t yaw,
                   int16_t motor_out[MIXER_NUM_MOTORS]);

#endif /* MIXER_H */
