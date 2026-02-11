/*
 * mixer.c — 6-thruster mixing matrix
 *
 * Converts 4 DOF commands into 6 motor outputs.
 *
 * Thruster layout (top-down, front = up):
 *
 *        FL ╲     ╱ FR        Horizontal (45° vectored)
 *             ╲ ╱
 *            [ROV]
 *             ╱ ╲
 *        BL ╱     ╲ BR
 *
 *         VL         VR        Vertical
 *
 * Mixing matrix:
 *              surge  sway  heave  yaw
 *   FR  [0]    +1     +1     0    +1
 *   FL  [1]    +1     -1     0    -1
 *   BR  [2]    -1     +1     0    -1
 *   BL  [3]    -1     -1     0    +1
 *   VL  [4]     0      0    +1     0
 *   VR  [5]     0      0    +1     0
 */

#include "mixer.h"

static inline int16_t clamp16(int32_t v, int16_t lo, int16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

void mixer_compute(int16_t surge, int16_t sway, int16_t heave, int16_t yaw,
                   int16_t motor_out[MIXER_NUM_MOTORS])
{
    /*
     * Raw mix — simple additive.  Divide by 2 so that full surge + full yaw
     * doesn't exceed ±1000 (each axis can contribute up to 500).
     */
    int32_t s = (int32_t)surge;
    int32_t w = (int32_t)sway;
    int32_t h = (int32_t)heave;
    int32_t y = (int32_t)yaw;

    int32_t raw[MIXER_NUM_MOTORS] = {
        ( s + w + y) / 2,     /* FR */
        ( s - w - y) / 2,     /* FL */
        (-s + w - y) / 2,     /* BR */
        (-s - w + y) / 2,     /* BL */
        h,                     /* VL */
        h,                     /* VR */
    };

    /* Normalise: find max magnitude, scale if > 1000 */
    int32_t max_mag = 0;
    for (int i = 0; i < MIXER_NUM_MOTORS; i++) {
        int32_t mag = raw[i] < 0 ? -raw[i] : raw[i];
        if (mag > max_mag) max_mag = mag;
    }

    if (max_mag > 1000 && max_mag != 0) {
        for (int i = 0; i < MIXER_NUM_MOTORS; i++)
            raw[i] = (int32_t)((int64_t)raw[i] * 1000 / max_mag);
    }

    for (int i = 0; i < MIXER_NUM_MOTORS; i++)
        motor_out[i] = clamp16(raw[i], -1000, 1000);
}
