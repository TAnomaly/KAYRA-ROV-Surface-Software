/*
 * keyboard.h — Keyboard input for test/fallback control
 *
 * When no joystick is connected (or for quick testing), maps keyboard
 * keys to the same joystick_state_t axes used by the rest of the system.
 *
 * Key mapping:
 *   W / S        — Surge  (forward / back)
 *   A / D        — Sway   (left / right)
 *   Q / E        — Yaw    (rotate left / right)
 *   Space / LShift — Heave (up / down)
 *
 * Keys produce full-scale ±1.0 (digital, not analog).
 */

#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdbool.h>
#include "joystick.h"   /* joystick_state_t */

#ifdef __cplusplus
extern "C" {
#endif

/*  Read SDL keyboard state and write axes/buttons into *state.
 *  Only overwrites axes that have active key input.
 *  Returns true if any control key is currently pressed.          */
bool keyboard_update(joystick_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* KEYBOARD_H */
