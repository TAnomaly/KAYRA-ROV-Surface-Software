/*
 * joystick.c — SDL2 joystick input with deadzone, normalization,
 *              and automatic reconnection.
 */

#include <SDL2/SDL.h>
#include <stdio.h>
#include <math.h>

#include "config.h"
#include "joystick.h"
#include "safety.h"   /* time_ms() */

static SDL_Joystick *g_js = NULL;

/* ------------------------------------------------------------------ */

int joystick_init(void)
{
    /* Init the joystick subsystem if the caller hasn't already
       (e.g. when running in GUI mode SDL is already initialised). */
    if (!(SDL_WasInit(0) & SDL_INIT_JOYSTICK)) {
        if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
            fprintf(stderr, "[joystick] SDL_Init failed: %s\n", SDL_GetError());
            return -1;
        }
    }

    int n = SDL_NumJoysticks();
    if (n < 1) {
        fprintf(stderr, "[joystick] No joystick found (%d detected)\n", n);
        return -1;
    }

    g_js = SDL_JoystickOpen(JOYSTICK_INDEX);
    if (!g_js) {
        fprintf(stderr, "[joystick] Failed to open index %d: %s\n",
                JOYSTICK_INDEX, SDL_GetError());
        return -1;
    }

    printf("[joystick] Opened: %s\n", SDL_JoystickName(g_js));
    printf("[joystick]   Axes: %d   Buttons: %d   Hats: %d\n",
           SDL_JoystickNumAxes(g_js),
           SDL_JoystickNumButtons(g_js),
           SDL_JoystickNumHats(g_js));

    return 0;
}

/* ------------------------------------------------------------------ */

/*
 * Deadzone with linear rescale so the output ramps smoothly from 0
 * just past the deadzone boundary up to +/-1.0 at full deflection.
 */
static float apply_deadzone(float v, float dz)
{
    float abs_v = fabsf(v);
    if (abs_v < dz)
        return 0.0f;
    float sign = (v > 0.0f) ? 1.0f : -1.0f;
    return sign * (abs_v - dz) / (1.0f - dz);
}

/* ------------------------------------------------------------------ */

void joystick_update(joystick_state_t *state)
{
    /* Update joystick internal state.  In GUI mode the event loop
       already pumps events; SDL_JoystickUpdate() is safe in both. */
    SDL_JoystickUpdate();

    /* Check connection */
    state->connected = (g_js != NULL && SDL_JoystickGetAttached(g_js));

    if (!state->connected) {
        /* Attempt reconnect */
        if (SDL_NumJoysticks() > JOYSTICK_INDEX) {
            g_js = SDL_JoystickOpen(JOYSTICK_INDEX);
            if (g_js) {
                printf("[joystick] Reconnected: %s\n",
                       SDL_JoystickName(g_js));
                state->connected = true;
            }
        }
        if (!state->connected) {
            for (int i = 0; i < JOYSTICK_MAX_AXES; i++)
                state->axes[i] = 0.0f;
            state->buttons = 0;
            return;
        }
    }

    /* Joystick is alive — timestamp the read */
    state->last_input_ms = time_ms();

    state->num_axes    = SDL_JoystickNumAxes(g_js);
    state->num_buttons = SDL_JoystickNumButtons(g_js);

    if (state->num_axes > JOYSTICK_MAX_AXES)
        state->num_axes = JOYSTICK_MAX_AXES;
    if (state->num_buttons > JOYSTICK_MAX_BUTTONS)
        state->num_buttons = JOYSTICK_MAX_BUTTONS;

    /* Read axes — SDL returns int16 [-32768, 32767] */
    for (int i = 0; i < state->num_axes; i++) {
        float norm = (float)SDL_JoystickGetAxis(g_js, i) / 32767.0f;

        /* Clamp: -32768/32767 ≈ -1.00003 */
        if (norm < -1.0f) norm = -1.0f;
        if (norm >  1.0f) norm =  1.0f;

        state->axes[i] = apply_deadzone(norm, JOYSTICK_DEADZONE);
    }

    /* Read buttons into bitmask */
    state->buttons = 0;
    for (int i = 0; i < state->num_buttons; i++) {
        if (SDL_JoystickGetButton(g_js, i))
            state->buttons |= (uint16_t)(1u << i);
    }
}

/* ------------------------------------------------------------------ */

void joystick_close(void)
{
    if (g_js) {
        SDL_JoystickClose(g_js);
        g_js = NULL;
    }
    /* Don't call SDL_Quit here — main owns the SDL lifecycle. */
}

bool joystick_is_connected(void)
{
    return g_js != NULL && SDL_JoystickGetAttached(g_js);
}
