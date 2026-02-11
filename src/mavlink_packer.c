/*
 * mavlink_packer.c — Convert joystick state into MAVLink frames
 *
 * Message choice rationale:
 *
 *   We use MANUAL_CONTROL (#69) rather than RC_CHANNELS_OVERRIDE (#70)
 *   because:
 *
 *   1. MANUAL_CONTROL is *purpose-built* for GCS joystick control.
 *      It carries four normalised axes (x, y, z, r) plus a button
 *      bitmask — exactly the abstraction a joystick provides.
 *
 *   2. RC_CHANNELS_OVERRIDE maps to PWM channel indices (ch1–ch18).
 *      This tightly couples the GCS to the vehicle's RC channel
 *      assignment, which varies per airframe and autopilot config.
 *      MANUAL_CONTROL is channel-agnostic.
 *
 *   3. ArduSub (the standard ROV firmware) natively accepts
 *      MANUAL_CONTROL and remaps it to thrusters internally.
 *      QGroundControl uses the same message for joystick input.
 *
 *   4. MANUAL_CONTROL values are normalised integers [-1000, 1000],
 *      making them independent of PWM calibration.
 *
 * Field mapping:
 *
 *   Joystick axis             → MAVLink field   → Meaning
 *   ─────────────────────────────────────────────────────────
 *   Left stick X  (axis 0)   → y [-1000,+1000] → sway  (left/right)
 *   Left stick Y  (axis 1)   → x [-1000,+1000] → surge (fwd/back)
 *   Right stick X (axis 2)   → r [-1000,+1000] → yaw   (CW/CCW)
 *   Right stick Y (axis 3)   → z [-1000,+1000] → heave (up/down)
 *   Buttons 0-15             → buttons bitmask
 */

#include <string.h>

#include "config.h"
#include "mavlink_minimal.h"
#include "mavlink_packer.h"

static uint8_t g_seq = 0;

/* ------------------------------------------------------------------ */

void mavlink_packer_init(void)
{
    g_seq = 0;
}

/* ------------------------------------------------------------------ */

static int16_t clamp_i16(int val, int lo, int hi)
{
    if (val < lo) return (int16_t)lo;
    if (val > hi) return (int16_t)hi;
    return (int16_t)val;
}

void mavlink_packer_map(const joystick_state_t *js, manual_control_t *ctrl)
{
    /*
     * Scale [-1.0 .. +1.0]  →  [-1000 .. +1000]
     * Inversion is applied per config.h flags so "push forward = positive".
     */
    float surge = js->axes[AXIS_IDX_SURGE];
    float sway  = js->axes[AXIS_IDX_SWAY];
    float yaw   = js->axes[AXIS_IDX_YAW];
    float heave = js->axes[AXIS_IDX_HEAVE];

    if (AXIS_INVERT_SURGE) surge = -surge;
    if (AXIS_INVERT_SWAY)  sway  = -sway;
    if (AXIS_INVERT_YAW)   yaw   = -yaw;
    if (AXIS_INVERT_HEAVE) heave = -heave;

    ctrl->x = clamp_i16((int)(surge * 1000.0f), AXIS_CLAMP_MIN, AXIS_CLAMP_MAX);
    ctrl->y = clamp_i16((int)(sway  * 1000.0f), AXIS_CLAMP_MIN, AXIS_CLAMP_MAX);
    ctrl->z = clamp_i16((int)(heave * 1000.0f), AXIS_CLAMP_MIN, AXIS_CLAMP_MAX);
    ctrl->r = clamp_i16((int)(yaw   * 1000.0f), AXIS_CLAMP_MIN, AXIS_CLAMP_MAX);

    ctrl->buttons = js->buttons;
}

/* ------------------------------------------------------------------ */

uint16_t mavlink_packer_pack_manual_control(const manual_control_t *ctrl,
                                            uint8_t *buf, size_t buf_len)
{
    if (buf_len < MAVLINK_MAX_PACKET_LEN)
        return 0;

    mavlink_message_t msg;
    mavlink_msg_manual_control_pack(&msg,
                                    GCS_SYSTEM_ID, GCS_COMPONENT_ID,
                                    TARGET_SYSTEM_ID,
                                    ctrl->x, ctrl->y, ctrl->z, ctrl->r,
                                    ctrl->buttons,
                                    &g_seq);

    return mavlink_msg_to_send_buffer(buf, &msg);
}

/* ------------------------------------------------------------------ */

uint16_t mavlink_packer_pack_heartbeat(uint8_t *buf, size_t buf_len)
{
    if (buf_len < MAVLINK_MAX_PACKET_LEN)
        return 0;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(&msg,
                               GCS_SYSTEM_ID, GCS_COMPONENT_ID,
                               MAV_TYPE_GCS,
                               MAV_AUTOPILOT_INVALID,
                               MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                               0,   /* custom_mode */
                               MAV_STATE_ACTIVE,
                               &g_seq);

    return mavlink_msg_to_send_buffer(buf, &msg);
}
