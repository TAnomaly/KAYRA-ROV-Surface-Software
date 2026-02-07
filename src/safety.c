/*
 * safety.c — Failsafe and watchdog logic
 *
 * Two independent failsafe triggers:
 *   1. Joystick disconnected  (SDL reports not attached)
 *   2. No successful read for FAILSAFE_TIMEOUT_MS  (soft watchdog)
 *
 * When either triggers, all control axes are forced to neutral (0)
 * and a one-shot warning is printed.  Recovery is automatic once
 * valid reads resume.
 */

#include <stdio.h>
#include <time.h>

#include "config.h"
#include "safety.h"

static bool     g_failsafe = false;

/* ------------------------------------------------------------------ */

uint64_t time_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL
         + (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* ------------------------------------------------------------------ */

void safety_init(void)
{
    g_failsafe = false;
}

/* ------------------------------------------------------------------ */

static void force_neutral(manual_control_t *ctrl)
{
    ctrl->x       = 0;
    ctrl->y       = 0;
    ctrl->z       = 0;
    ctrl->r       = 0;
    ctrl->buttons = 0;
}

void safety_update(const joystick_state_t *js, manual_control_t *ctrl)
{
    uint64_t now = time_ms();

    /* Trigger 1: joystick not attached */
    if (!js->connected) {
        if (!g_failsafe)
            fprintf(stderr,
                    "\n[safety] FAILSAFE — joystick disconnected\n");
        g_failsafe = true;
        force_neutral(ctrl);
        return;
    }

    /* Trigger 2: no successful read within timeout */
    uint64_t age = now - js->last_input_ms;
    if (age > FAILSAFE_TIMEOUT_MS) {
        if (!g_failsafe)
            fprintf(stderr,
                    "\n[safety] FAILSAFE — no input for %lu ms\n",
                    (unsigned long)age);
        g_failsafe = true;
        force_neutral(ctrl);
        return;
    }

    /* All clear */
    if (g_failsafe) {
        printf("\n[safety] Failsafe cleared — resuming control\n");
        g_failsafe = false;
    }
}

bool safety_is_failsafe(void)
{
    return g_failsafe;
}
