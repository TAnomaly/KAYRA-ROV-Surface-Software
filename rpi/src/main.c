/*
 * main.c — KAYRA ROV onboard controller (Raspberry Pi 5)
 *
 * Main loop:
 *   1. Receive MAVLink MANUAL_CONTROL from GCS via UDP
 *   2. Mix 4 DOF channels → 6 motor outputs
 *   3. Write PWM to PCA9685 → ESCs → motors
 *   4. Send heartbeat + telemetry back to GCS
 *   5. Launch camera H264 stream in background
 *
 * Usage:
 *   ./kayra-rov                    (defaults)
 *   ./kayra-rov --no-camera        (skip camera stream)
 *   ./kayra-rov --no-pwm           (dry-run, no motor output)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "config.h"
#include "mavlink_parser.h"
#include "pca9685.h"
#include "mixer.h"
#include "camera.h"
#include "mpu6050.h"

/* ================================================================== */
/*  Globals                                                            */
/* ================================================================== */

static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig) { (void)sig; g_running = 0; }

/* ── Xbox controller button bits (SDL mapping) ── */
#define BTN_A       (1 << 0)
#define BTN_B       (1 << 1)
#define BTN_X       (1 << 2)
#define BTN_Y       (1 << 3)
#define BTN_LB      (1 << 4)
#define BTN_RB      (1 << 5)
#define BTN_BACK    (1 << 6)
#define BTN_START   (1 << 7)

/* Monotonic ms clock */
static uint64_t time_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

/* Map [-1000, +1000] → [PWM_MIN_US, PWM_MAX_US] with neutral at 1500 */
static int motor_to_pwm_us(int16_t val)
{
    /* Dead band: values close to 0 → 1000us pulse (motor stopped, ESC stays armed) */
    if (val > -30 && val < 30) return 1000;

    /* val: -1000..+1000 → 1100..1900 */
    int us = PWM_NEUTRAL_US + (int)val * (PWM_MAX_US - PWM_NEUTRAL_US) / 1000;
    if (us < PWM_MIN_US) us = PWM_MIN_US;
    if (us > PWM_MAX_US) us = PWM_MAX_US;
    return us;
}

/* ── Smooth ramping ── */

static float move_toward(float current, float target, float max_delta)
{
    float diff = target - current;
    if (diff >  max_delta) return current + max_delta;
    if (diff < -max_delta) return current - max_delta;
    return target;
}

/* ================================================================== */
/*  Main                                                               */
/* ================================================================== */

int main(int argc, char *argv[])
{
    /* ── Parse args ── */
    int use_camera = 1;
    int use_pwm    = 1;
    int use_imu    = 1;
    const char *gcs_ip   = GCS_IP;
    int         gcs_port = GCS_PORT;
    int         listen_port = LISTEN_PORT;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--no-camera") == 0) { use_camera = 0; continue; }
        if (strcmp(argv[i], "--no-pwm") == 0)    { use_pwm = 0;    continue; }
        if (strcmp(argv[i], "--no-imu") == 0)    { use_imu = 0;    continue; }
        if (strcmp(argv[i], "--gcs") == 0 && i + 1 < argc) {
            gcs_ip = argv[++i]; continue;
        }
        if (strcmp(argv[i], "--gcs-port") == 0 && i + 1 < argc) {
            gcs_port = atoi(argv[++i]); continue;
        }
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            listen_port = atoi(argv[++i]); continue;
        }
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("KAYRA ROV — Onboard Controller (RPi 5)\n\n"
                   "Usage: %s [OPTIONS]\n\n"
                   "  --no-camera          Skip camera streaming\n"
                   "  --no-pwm             Dry-run (no motor output)\n"
                   "  --no-imu             Skip IMU (MPU6050) reading\n"
                   "  --gcs <ip>           GCS IP address       (default: %s)\n"
                   "  --gcs-port <port>    GCS telemetry port   (default: %d)\n"
                   "  --port <port>        Listen port           (default: %d)\n",
                   argv[0], GCS_IP, GCS_PORT, LISTEN_PORT);
            return 0;
        }
    }

    /* ── Signals ── */
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    printf("╔══════════════════════════════════════════════╗\n");
    printf("║   KAYRA ROV — Onboard Controller              ║\n");
    printf("║   Listen : 0.0.0.0:%-5d                      ║\n", listen_port);
    printf("║   GCS    : %s:%-5d                    ║\n", gcs_ip, gcs_port);
    printf("║   Motors : %-3s   Camera : %-3s                 ║\n",
           use_pwm ? "ON" : "OFF", use_camera ? "ON" : "OFF");
    printf("╚══════════════════════════════════════════════╝\n\n");

    /* ── UDP socket (non-blocking) ── */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("[main] socket"); return 1; }

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons((uint16_t)listen_port);
    bind_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        perror("[main] bind");
        close(sock);
        return 1;
    }

    /* Non-blocking so main loop doesn't stall */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 1000 }; /* 1ms timeout */
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* GCS address for sending telemetry back */
    struct sockaddr_in gcs_addr;
    memset(&gcs_addr, 0, sizeof(gcs_addr));
    gcs_addr.sin_family = AF_INET;
    gcs_addr.sin_port   = htons((uint16_t)gcs_port);
    inet_pton(AF_INET, gcs_ip, &gcs_addr.sin_addr);

    /* ── PCA9685 ── */
    pca9685_t pwm;
    memset(&pwm, 0, sizeof(pwm));
    pwm.fd = -1;

    if (use_pwm) {
        if (pca9685_init(&pwm, PCA9685_I2C_BUS, PCA9685_I2C_ADDR,
                         PCA9685_FREQ_HZ) < 0) {
            fprintf(stderr, "[main] PCA9685 init failed — running without PWM\n");
            use_pwm = 0;
        } else {
            /* Arm ESCs: send neutral pulse for several seconds.
             * ESCs need to see 1500us (neutral) at power-on to arm.
             * We send it in a loop to ensure stable signal.        */
            printf("[main] Arming ESCs — sending neutral (5s)...\n");
            for (int t = 0; t < 50; t++) {   /* 50 x 100ms = 5 seconds */
                pca9685_set_all_us(&pwm, PWM_NEUTRAL_US);
                usleep(100000);
            }
            printf("[main] ESCs armed. Bip sesi duyduysan hazir.\n");
        }
    }

    /* ── IMU (MPU6050) ── */
    mpu6050_t imu;
    memset(&imu, 0, sizeof(imu));
    imu.fd = -1;

    if (use_imu) {
        if (mpu6050_init(&imu, PCA9685_I2C_BUS, MPU6050_ADDR) < 0) {
            fprintf(stderr, "[main] IMU init failed — running without IMU\n");
            use_imu = 0;
        }
    }

    /* ── Camera ── */
    pid_t cam_pid = -1;
    if (use_camera) {
        cam_pid = camera_start(gcs_ip, CAMERA_PORT,
                               CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS);
    }

    /* ── MAVLink parser ── */
    mavlink_parser_t parser;
    mavlink_parser_init(&parser);

    /* ── State ── */
    manual_control_msg_t ctrl;
    memset(&ctrl, 0, sizeof(ctrl));

    int16_t motors[MIXER_NUM_MOTORS] = {0};
    float   smooth[MIXER_NUM_MOTORS] = {0};   /* ramped motor values */

    uint64_t last_packet_ms  = time_ms();
    uint64_t last_hb_ms      = 0;
    uint64_t last_telem_ms   = 0;
    uint64_t last_print_ms   = 0;
    uint64_t packets_recv    = 0;
    uint8_t  mav_seq         = 0;
    int      failsafe        = 0;
    int      armed           = 0;       /* 0 = disarmed, 1 = armed */
    uint16_t prev_buttons    = 0;       /* for edge detection */

    uint64_t hz_start  = time_ms();
    uint64_t hz_frames = 0;
    float    loop_hz   = 0;

    printf("[main] Entering main loop at %d Hz\n\n", MAIN_LOOP_HZ);

    /* ── Main loop ── */
    while (g_running) {
        uint64_t now = time_ms();

        /* ── 1. Receive UDP data ── */
        uint8_t rxbuf[512];
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);
        ssize_t n = recvfrom(sock, rxbuf, sizeof(rxbuf), 0,
                             (struct sockaddr *)&sender_addr, &sender_len);

        if (n > 0) {
            /* Auto-detect GCS IP from incoming packets */
            if (gcs_addr.sin_addr.s_addr != sender_addr.sin_addr.s_addr) {
                gcs_addr.sin_addr = sender_addr.sin_addr;
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &sender_addr.sin_addr, ip_str, sizeof(ip_str));
                printf("\n[GCS] Auto-detected GCS IP: %s (telemetry → %s:%d)\n",
                       ip_str, ip_str, gcs_port);
            }

            for (ssize_t i = 0; i < n; i++) {
                if (mavlink_parser_feed(&parser, rxbuf[i])) {
                    manual_control_msg_t mc;
                    if (mavlink_parser_get_manual_control(&parser, &mc)) {
                        ctrl = mc;
                        last_packet_ms = now;
                        packets_recv++;
                        failsafe = 0;
                    }
                }
            }
        }

        /* ── 2. Failsafe check ── */
        if (now - last_packet_ms > FAILSAFE_TIMEOUT_MS) {
            if (!failsafe) {
                fprintf(stderr, "\n[FAILSAFE] No packets for %d ms — MOTORS STOP\n",
                        FAILSAFE_TIMEOUT_MS);
                failsafe = 1;
                /* Motors already forced to 0 below — keep armed state
                 * so control resumes automatically when packets return.
                 * ARM/DISARM is only changed by the START button.       */
            }
            ctrl.x = 0; ctrl.y = 0; ctrl.z = 0; ctrl.r = 0;
            ctrl.buttons = 0;
        }

        /* ── 2b. Button edge detection ── */
        uint16_t pressed = ctrl.buttons & ~prev_buttons;  /* newly pressed */
        prev_buttons = ctrl.buttons;

        /* START button → toggle ARM/DISARM */
        if (pressed & BTN_START) {
            armed = !armed;
            if (armed) {
                printf("\n[ARM] Motors ARMED — joystick active\n");
            } else {
                printf("\n[DISARM] Motors DISARMED — all neutral\n");
                /* Immediately force all motors to neutral */
                if (use_pwm)
                    pca9685_set_all_us(&pwm, PWM_NEUTRAL_US);
                for (int i = 0; i < MIXER_NUM_MOTORS; i++)
                    smooth[i] = 0.0f;
            }
        }

        /* ── 3. Mix → motors ── */
        if (armed && !failsafe) {
            /* X button held → all motors full forward (test) */
            if (ctrl.buttons & BTN_X) {
                for (int i = 0; i < MIXER_NUM_MOTORS; i++)
                    motors[i] = 1000;
            } else {
                mixer_compute(ctrl.x, ctrl.y, ctrl.z, ctrl.r, motors);
            }
        } else {
            /* DISARMED or FAILSAFE → all motors zero */
            for (int i = 0; i < MIXER_NUM_MOTORS; i++)
                motors[i] = 0;
        }

        /* ── 3b. Smooth ramp (skip ramp when disarmed — instant stop) ── */
        {
            static uint64_t prev_ms = 0;
            if (prev_ms == 0) prev_ms = now;
            float dt = (float)(now - prev_ms) / 1000.0f;
            if (dt > 0.1f) dt = 0.1f;
            prev_ms = now;

            if (!armed || failsafe) {
                /* INSTANT STOP — no ramping */
                for (int i = 0; i < MIXER_NUM_MOTORS; i++)
                    smooth[i] = 0.0f;
            } else {
                float max_step = MOTOR_RAMP_RATE * dt;
                for (int i = 0; i < MIXER_NUM_MOTORS; i++)
                    smooth[i] = move_toward(smooth[i], (float)motors[i], max_step);
            }
        }

        /* ── 4. Write PWM ── */
        if (use_pwm) {
            static const int ch_map[NUM_MOTORS] = {
                MOTOR_CH_FR, MOTOR_CH_FL, MOTOR_CH_BR,
                MOTOR_CH_BL, MOTOR_CH_VL, MOTOR_CH_VR,
            };
            if (!armed || failsafe) {
                /* DISARMED / FAILSAFE → sinyal tamamen kes (0us = no pulse)
                 * ESC "no signal" moduna gecer, motor kesinlikle donmez */
                pca9685_set_all_us(&pwm, 0);
            } else {
                for (int i = 0; i < NUM_MOTORS; i++) {
                    pca9685_set_pulse_us(&pwm, ch_map[i],
                                         motor_to_pwm_us((int16_t)smooth[i]));
                }
            }
        }

        /* ── 5. Send heartbeat at 1 Hz ── */
        if (now - last_hb_ms >= HEARTBEAT_INTERVAL_MS) {
            uint8_t hbbuf[64];
            int len = mavlink_pack_heartbeat(hbbuf, sizeof(hbbuf), &mav_seq, armed);
            if (len > 0) {
                sendto(sock, hbbuf, (size_t)len, 0,
                       (struct sockaddr *)&gcs_addr, sizeof(gcs_addr));
            }
            last_hb_ms = now;
        }

        /* ── 5b. Read IMU ── */
        if (use_imu)
            mpu6050_update(&imu);

        /* ── 5c. Send telemetry at 10 Hz ── */
        if (now - last_telem_ms >= TELEMETRY_INTERVAL_MS) {
            uint32_t boot_ms = (uint32_t)(now & 0xFFFFFFFF);
            uint8_t tbuf[64];
            int tlen;

            /* ATTITUDE — real IMU data (degrees → radians) */
            float roll_rad  = use_imu ? imu.roll  * ((float)M_PI / 180.0f) : 0.0f;
            float pitch_rad = use_imu ? imu.pitch * ((float)M_PI / 180.0f) : 0.0f;
            float yaw_rad   = use_imu ? imu.yaw   * ((float)M_PI / 180.0f) : 0.0f;
            tlen = mavlink_pack_attitude(tbuf, sizeof(tbuf), &mav_seq,
                       boot_ms, roll_rad, pitch_rad, yaw_rad, 0.0f, 0.0f, 0.0f);
            if (tlen > 0)
                sendto(sock, tbuf, (size_t)tlen, 0,
                       (struct sockaddr *)&gcs_addr, sizeof(gcs_addr));

            /* SYS_STATUS — placeholder (real ADC would fill these) */
            tlen = mavlink_pack_sys_status(tbuf, sizeof(tbuf), &mav_seq,
                       12600, 320, 85);  /* 12.6V, 3.2A, 85% */
            if (tlen > 0)
                sendto(sock, tbuf, (size_t)tlen, 0,
                       (struct sockaddr *)&gcs_addr, sizeof(gcs_addr));

            /* SCALED_PRESSURE — depth placeholder, temp from IMU */
            int16_t temp_centi = use_imu ? (int16_t)(imu.temp_c * 100.0f) : 2500;
            tlen = mavlink_pack_scaled_pressure(tbuf, sizeof(tbuf), &mav_seq,
                       boot_ms, 1013.25f, 0.0f, temp_centi);
            if (tlen > 0)
                sendto(sock, tbuf, (size_t)tlen, 0,
                       (struct sockaddr *)&gcs_addr, sizeof(gcs_addr));

            last_telem_ms = now;
        }

        /* ── 6. Console status (4 Hz) ── */
        if (now - last_print_ms >= 250) {
            printf("\r  [%s|%s] x:%+5d y:%+5d z:%+5d r:%+5d | "
                   "R:%+6.1f P:%+6.1f Y:%5.1f %4.1fC | "
                   "pkts:%lu %.0fHz    ",
                   failsafe ? "FAIL" : " OK ",
                   armed    ? " ARM" : "DSRM",
                   ctrl.x, ctrl.y, ctrl.z, ctrl.r,
                   use_imu ? imu.roll  : 0.0f,
                   use_imu ? imu.pitch : 0.0f,
                   use_imu ? imu.yaw   : 0.0f,
                   use_imu ? imu.temp_c : 0.0f,
                   (unsigned long)packets_recv, loop_hz);
            fflush(stdout);
            last_print_ms = now;
        }

        /* ── 7. Hz counter ── */
        hz_frames++;
        if (now > hz_start && (now - hz_start) >= 1000) {
            uint64_t elapsed = now - hz_start;
            loop_hz = (elapsed > 0) ? (float)hz_frames * 1000.0f / (float)elapsed
                                    : 0.0f;
            hz_start = now;
            hz_frames = 0;
        }

        /* ── 8. Rate limit ── */
        usleep(MAIN_LOOP_US);
    }

    /* ── Shutdown ── */
    printf("\n\n[main] Shutting down...\n");

    /* Neutral motors */
    if (use_pwm) {
        printf("[main] Motors → neutral\n");
        pca9685_set_all_us(&pwm, PWM_NEUTRAL_US);
        usleep(100000);
        pca9685_close(&pwm);
    }

    /* Stop IMU */
    if (use_imu)
        mpu6050_close(&imu);

    /* Stop camera */
    if (cam_pid > 0)
        camera_stop(cam_pid);

    close(sock);
    printf("[main] Done.\n");
    return 0;
}
