/*
 * keyboard.c — Keyboard fallback input for testing
 *
 * Uses SDL_GetKeyboardState() to read currently pressed keys and maps
 * them to joystick axes with smooth acceleration / deceleration ramps
 * so control feels analog rather than binary on/off.
 *
 *   KAYRA ROV layout (left hand = left stick, right hand = right stick):
 *
 *   W / S              → Surge  (left stick Y — forward/back)
 *   A / D              → Sway   (left stick X — strafe left/right)
 *   Q / E              → Yaw    (right stick X — turn left/right)
 *   Space / Left Shift → Heave  (right stick Y — ascend/descend)
 */

#include <SDL2/SDL.h>
#include <math.h>
#include "config.h"
#include "joystick.h"
#include "keyboard.h"
#include "safety.h"   /* time_ms() */

/* ── Ramp tuning ──────────────────────────────────────────────── */
#define KB_ACCEL_RATE   2.0f   /* units/sec — time to reach full  */
#define KB_DECEL_RATE   4.0f   /* units/sec — time to return to 0 */
#define KB_MAX_VAL      1.0f

/* Persistent axis state (survives across calls). */
static float s_surge = 0.0f;
static float s_sway  = 0.0f;
static float s_yaw   = 0.0f;
static float s_heave = 0.0f;
static uint64_t s_last_ms = 0;

/* Move *val toward target by at most step (always positive). */
static float move_toward(float val, float target, float step)
{
    if (val < target) {
        val += step;
        if (val > target) val = target;
    } else if (val > target) {
        val -= step;
        if (val < target) val = target;
    }
    return val;
}

bool keyboard_update(joystick_state_t *state)
{
    const Uint8 *k = SDL_GetKeyboardState(NULL);
    if (!k) return false;

    /* Delta-time in seconds */
    uint64_t now = time_ms();
    float dt = (s_last_ms == 0) ? 0.02f : (float)(now - s_last_ms) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;   /* cap after pause */
    s_last_ms = now;

    /* Determine target for each axis based on key state */
    float tgt_surge = 0.0f, tgt_sway = 0.0f;
    float tgt_yaw   = 0.0f, tgt_heave = 0.0f;

    if (k[SDL_SCANCODE_W])      tgt_surge += KB_MAX_VAL;
    if (k[SDL_SCANCODE_S])      tgt_surge -= KB_MAX_VAL;
    if (k[SDL_SCANCODE_D])      tgt_sway  += KB_MAX_VAL;  /* left stick X */
    if (k[SDL_SCANCODE_A])      tgt_sway  -= KB_MAX_VAL;
    if (k[SDL_SCANCODE_E])      tgt_yaw   += KB_MAX_VAL;  /* right stick X */
    if (k[SDL_SCANCODE_Q])      tgt_yaw   -= KB_MAX_VAL;
    if (k[SDL_SCANCODE_SPACE])  tgt_heave += KB_MAX_VAL;
    if (k[SDL_SCANCODE_LSHIFT]) tgt_heave -= KB_MAX_VAL;

    /* Ramp each axis smoothly toward its target */
    float accel = KB_ACCEL_RATE * dt;
    float decel = KB_DECEL_RATE * dt;

    s_surge = move_toward(s_surge, tgt_surge, tgt_surge != 0.0f ? accel : decel);
    s_sway  = move_toward(s_sway,  tgt_sway,  tgt_sway  != 0.0f ? accel : decel);
    s_yaw   = move_toward(s_yaw,   tgt_yaw,   tgt_yaw   != 0.0f ? accel : decel);
    s_heave = move_toward(s_heave, tgt_heave,  tgt_heave != 0.0f ? accel : decel);

    /* Check if any axis is still active (ramping or held) */
    bool active = fabsf(s_surge) > 0.001f || fabsf(s_sway) > 0.001f ||
                  fabsf(s_yaw)   > 0.001f || fabsf(s_heave) > 0.001f;

    if (!active) return false;

    /* Apply inversion to match joystick conventions */
    float surge = AXIS_INVERT_SURGE ? -s_surge : s_surge;
    float sway  = AXIS_INVERT_SWAY  ? -s_sway  : s_sway;
    float yaw   = AXIS_INVERT_YAW   ? -s_yaw   : s_yaw;
    float heave = AXIS_INVERT_HEAVE  ? -s_heave : s_heave;

    /* Write into the same axis slots the joystick uses */
    state->axes[AXIS_IDX_SURGE] = surge;
    state->axes[AXIS_IDX_SWAY]  = sway;
    state->axes[AXIS_IDX_YAW]   = yaw;
    state->axes[AXIS_IDX_HEAVE] = heave;

    state->num_axes      = JOYSTICK_MAX_AXES;
    state->num_buttons   = 0;
    state->connected     = true;          /* virtual "connection" */
    state->last_input_ms = time_ms();

    return true;
}
