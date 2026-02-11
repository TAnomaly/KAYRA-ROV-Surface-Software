/*
 * config.h — Tuneable parameters for KAYRA ROV onboard controller (RPi 5)
 */

#ifndef RPI_CONFIG_H
#define RPI_CONFIG_H

/* ── Network ── */
#define LISTEN_PORT         14550           /* UDP port to receive MAVLink   */
#define GCS_IP              "192.168.2.1"   /* PC ground station address     */
#define GCS_PORT            14551           /* port to send telemetry back   */
#define CAMERA_PORT         5600            /* H264 RTP stream port on PC    */

/* ── MAVLink IDs ── */
#define VEHICLE_SYSID       1
#define VEHICLE_COMPID      1
#define GCS_SYSID           255

/* ── Timing ── */
#define MAIN_LOOP_HZ        200
#define MAIN_LOOP_US        (1000000 / MAIN_LOOP_HZ)
#define HEARTBEAT_INTERVAL_MS   1000
#define TELEMETRY_INTERVAL_MS   100         /* 10 Hz telemetry              */
#define FAILSAFE_TIMEOUT_MS     1000        /* neutral if no packet for 1s  */
#define WATCHDOG_TIMEOUT_MS     3000        /* reboot ESCs if 3s silence    */

/* ── PCA9685 PWM ── */
#define PCA9685_I2C_BUS     1               /* /dev/i2c-1 on RPi            */
#define PCA9685_I2C_ADDR    0x40            /* default PCA9685 address       */
#define PCA9685_FREQ_HZ     50              /* 50 Hz for ESCs               */
#define PWM_NEUTRAL_US      1500            /* ESC neutral pulse width       */
#define PWM_MIN_US          1100            /* ESC minimum pulse width       */
#define PWM_MAX_US          1900            /* ESC maximum pulse width       */
#define PWM_ARM_US          1500            /* arm pulse (send on startup)   */

/* ── Motor channels on PCA9685 ── */
#define NUM_MOTORS          6
#define MOTOR_CH_FR         0               /* Front-Right horizontal        */
#define MOTOR_CH_FL         1               /* Front-Left  horizontal        */
#define MOTOR_CH_BR         2               /* Back-Right  horizontal        */
#define MOTOR_CH_BL         3               /* Back-Left   horizontal        */
#define MOTOR_CH_VL         4               /* Vertical Left                 */
#define MOTOR_CH_VR         5               /* Vertical Right                */

/* ── Camera ── */
#define CAMERA_WIDTH        640
#define CAMERA_HEIGHT       480
#define CAMERA_FPS          30

/* ── Smooth ramping ── */
#define MOTOR_RAMP_RATE     8000.0f  /* units/sec — 0.25s from -1000 to +1000 */

/* ── Safety ── */
#define MOTOR_CLAMP_MIN     (-1000)
#define MOTOR_CLAMP_MAX     1000

#endif /* RPI_CONFIG_H */
