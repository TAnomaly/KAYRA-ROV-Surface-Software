/*
 * joystick.h — SDL2 joystick input with deadzone and normalization
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef struct {
    float    axes[JOYSTICK_MAX_AXES];   /* normalized to [-1.0, 1.0] */
    uint16_t buttons;                   /* bitmask, bit N = button N */
    int      num_axes;
    int      num_buttons;
    bool     connected;
    uint64_t last_input_ms;             /* timestamp of last good read */
} joystick_state_t;

/*  Returns 0 on success, -1 if no joystick found (non-fatal). */
int  joystick_init(void);

/*  Poll the joystick and fill *state.  Safe to call even when
 *  disconnected — axes are zeroed and connected=false.            */
void joystick_update(joystick_state_t *state);

void joystick_close(void);
bool joystick_is_connected(void);

#endif /* JOYSTICK_H */
