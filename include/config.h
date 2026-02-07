/*
 * config.h — Tuneable parameters for Surface Control GCS
 *
 * Edit these values to match your hardware and network setup.
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ---- Transport defaults ---- */
#define DEFAULT_TARGET_IP       "192.168.2.1"   /* Raspberry Pi on ROV   */
#define DEFAULT_UDP_PORT        14550           /* MAVLink standard port */
#define DEFAULT_SERIAL_DEVICE   "/dev/ttyUSB0"
#define DEFAULT_SERIAL_BAUD     115200

/* ---- Joystick ---- */
#define JOYSTICK_INDEX          0               /* SDL joystick index    */
#define JOYSTICK_DEADZONE       0.05f           /* 5 % center deadzone   */
#define JOYSTICK_MAX_AXES       8
#define JOYSTICK_MAX_BUTTONS    16

/*
 * Axis mapping — indices into SDL axis array.
 * Adjust these for your specific controller.
 *
 * Defaults assume a standard dual-stick gamepad:
 *   Left  stick X = axis 0    Left  stick Y = axis 1
 *   Right stick X = axis 2    Right stick Y = axis 3
 */
#define AXIS_IDX_SURGE          1   /* left stick Y  → forward/back  */
#define AXIS_IDX_SWAY           0   /* left stick X  → left/right    */
#define AXIS_IDX_YAW            2   /* right stick X → yaw           */
#define AXIS_IDX_HEAVE          3   /* right stick Y → up/down       */

#define AXIS_INVERT_SURGE       1   /* 1 = invert (push-up = fwd)    */
#define AXIS_INVERT_SWAY        0
#define AXIS_INVERT_YAW         0
#define AXIS_INVERT_HEAVE       1   /* 1 = invert (push-up = ascend) */

/* ---- MAVLink IDs ---- */
#define GCS_SYSTEM_ID           255 /* standard GCS sysid            */
#define GCS_COMPONENT_ID        0
#define TARGET_SYSTEM_ID        1   /* vehicle sysid                 */
#define TARGET_COMPONENT_ID     0

/* ---- Timing ---- */
#define LOOP_RATE_HZ            50
#define LOOP_PERIOD_MS          (1000 / LOOP_RATE_HZ)
#define HEARTBEAT_INTERVAL_MS   1000
#define FAILSAFE_TIMEOUT_MS     500
#define STATUS_PRINT_INTERVAL_MS 250

/* ---- Safety clamps ---- */
#define AXIS_CLAMP_MIN          (-1000)
#define AXIS_CLAMP_MAX          1000

#endif /* CONFIG_H */
