/*
 * main.c — Surface Control GCS main loop
 *
 * Supports two modes:
 *   CLI mode (default):  text status line on the terminal
 *   GUI mode (--gui):    Dear ImGui visual dashboard
 *
 * Data flow (identical in both modes):
 *   Joystick → SDL2 → normalize/deadzone → MAVLink MANUAL_CONTROL
 *            → UDP or Serial → Raspberry Pi on the ROV
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>

#include <SDL2/SDL.h>

#include "config.h"
#include "joystick.h"
#include "mavlink_packer.h"
#include "mavlink_minimal.h"
#include "transport.h"
#include "safety.h"
#include "keyboard.h"
#include "gui.h"
#include "telemetry_rx.h"

/* ------------------------------------------------------------------ */

static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ------------------------------------------------------------------ */

static void print_usage(const char *prog)
{
    printf(
        "Surface Control — ROV Ground Station\n"
        "\n"
        "Usage: %s [OPTIONS]\n"
        "\n"
        "  -g, --gui                     Launch visual dashboard\n"
        "  -t, --transport <udp|serial>  Transport type     (default: udp)\n"
        "  -i, --ip <address>            Target IP address  (default: %s)\n"
        "  -p, --port <port>             UDP port           (default: %d)\n"
        "  -d, --device <path>           Serial device      (default: %s)\n"
        "  -b, --baud <rate>             Serial baud rate   (default: %d)\n"
        "  -h, --help                    Show this help\n",
        prog,
        DEFAULT_TARGET_IP,
        DEFAULT_UDP_PORT,
        DEFAULT_SERIAL_DEVICE,
        DEFAULT_SERIAL_BAUD);
}

/* ------------------------------------------------------------------ */

static void print_status(const joystick_state_t *js,
                         const manual_control_t  *ctrl,
                         bool failsafe)
{
    printf("\r  JS [%s]  x:%+5d  y:%+5d  z:%+5d  r:%+5d  "
           "btn:0x%04X  %s    ",
           js->connected ? " OK " : "LOST",
           ctrl->x, ctrl->y, ctrl->z, ctrl->r,
           ctrl->buttons,
           failsafe ? "** FAILSAFE **" : "");
    fflush(stdout);
}

/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
    /* ---- defaults ---- */
    int gui_mode = 0;

    transport_t transport;
    memset(&transport, 0, sizeof(transport));
    transport.type        = TRANSPORT_UDP;
    transport.fd          = -1;
    transport.target_port = DEFAULT_UDP_PORT;
    transport.listen_port = DEFAULT_LISTEN_PORT;
    transport.baud_rate   = DEFAULT_SERIAL_BAUD;
    strncpy(transport.target_ip, DEFAULT_TARGET_IP,
            sizeof(transport.target_ip) - 1);
    strncpy(transport.device, DEFAULT_SERIAL_DEVICE,
            sizeof(transport.device) - 1);

    /* ---- parse CLI args ---- */
    static struct option long_opts[] = {
        { "gui",       no_argument,       NULL, 'g' },
        { "transport", required_argument, NULL, 't' },
        { "ip",        required_argument, NULL, 'i' },
        { "port",      required_argument, NULL, 'p' },
        { "device",    required_argument, NULL, 'd' },
        { "baud",      required_argument, NULL, 'b' },
        { "help",      no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "gt:i:p:d:b:h",
                              long_opts, NULL)) != -1) {
        switch (opt) {
        case 'g':
            gui_mode = 1;
            break;
        case 't':
            if (strcmp(optarg, "serial") == 0)
                transport.type = TRANSPORT_SERIAL;
            else
                transport.type = TRANSPORT_UDP;
            break;
        case 'i':
            strncpy(transport.target_ip, optarg,
                    sizeof(transport.target_ip) - 1);
            break;
        case 'p':
            transport.target_port = atoi(optarg);
            break;
        case 'd':
            strncpy(transport.device, optarg,
                    sizeof(transport.device) - 1);
            break;
        case 'b':
            transport.baud_rate = atoi(optarg);
            break;
        case 'h':
            print_usage(argv[0]);
            return 0;
        default:
            print_usage(argv[0]);
            return 1;
        }
    }

    /* ---- signals ---- */
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    /* ---- banner ---- */
    printf("=== Surface Control — ROV Ground Station ===\n");
    printf("    Mode: %s\n\n", gui_mode ? "GUI" : "CLI");

    /* ---- SDL init ----
     * In GUI mode SDL_INIT_VIDEO is required for the window.
     * In CLI mode we only need the joystick subsystem.
     * joystick_init() will skip SDL_Init if already done.       */
    Uint32 sdl_flags = SDL_INIT_JOYSTICK;
    if (gui_mode)
        sdl_flags |= SDL_INIT_VIDEO | SDL_INIT_TIMER;

    if (SDL_Init(sdl_flags) < 0) {
        fprintf(stderr, "[main] SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    /* ---- GUI init (before joystick so window exists) ---- */
    if (gui_mode) {
        if (gui_init(1280, 800) < 0) {
            fprintf(stderr, "[main] GUI init failed\n");
            SDL_Quit();
            return 1;
        }
    }

    /* ---- subsystems ---- */
    if (joystick_init() < 0)
        fprintf(stderr, "[main] No joystick at startup — will retry\n");

    mavlink_packer_init();
    safety_init();

    /* Telemetry receiver (for incoming data from RPi) */
    telemetry_parser_t telem_parser;
    telemetry_state_t  telem;
    telemetry_rx_init(&telem_parser, &telem);

    if (transport_init(&transport) < 0) {
        fprintf(stderr, "[main] Transport init failed — aborting\n");
        joystick_close();
        if (gui_mode) gui_shutdown();
        SDL_Quit();
        return 1;
    }

    printf("[main] Loop rate: %d Hz  Failsafe timeout: %d ms\n\n",
           LOOP_RATE_HZ, FAILSAFE_TIMEOUT_MS);

    /* ---- build transport label for GUI ---- */
    char transport_str[300];
    if (transport.type == TRANSPORT_UDP)
        snprintf(transport_str, sizeof(transport_str),
                 "UDP  %s:%d", transport.target_ip, transport.target_port);
    else
        snprintf(transport_str, sizeof(transport_str),
                 "Serial  %s @ %d", transport.device, transport.baud_rate);

    /* ---- main loop ---- */
    joystick_state_t js;
    memset(&js, 0, sizeof(js));
    js.last_input_ms = time_ms();

    manual_control_t ctrl;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint64_t last_heartbeat = 0;
    uint64_t last_status    = 0;
    uint64_t packets_sent   = 0;

    /* Simple Hz estimator */
    uint64_t hz_start   = time_ms();
    uint64_t hz_frames  = 0;
    float    loop_hz    = 0.0f;

    while (g_running) {
        uint64_t now = time_ms();

        /* 1 — read joystick */
        joystick_update(&js);

        /* 1b — keyboard fallback (when joystick is disconnected or
         *      for quick testing — keys override axes directly) */
        if (!js.connected)
            keyboard_update(&js);

        /* 1c — smooth ramp on all axes (prevents abrupt jumps) */
        {
            static float smooth_axes[JOYSTICK_MAX_AXES] = {0};
            static uint64_t prev_ms = 0;
            if (prev_ms == 0) prev_ms = now;
            float dt = (float)(now - prev_ms) / 1000.0f;
            if (dt > 0.1f) dt = 0.1f;
            float max_step = JS_RAMP_RATE * dt;
            for (int i = 0; i < JOYSTICK_MAX_AXES; i++) {
                float diff = js.axes[i] - smooth_axes[i];
                if (diff >  max_step) smooth_axes[i] += max_step;
                else if (diff < -max_step) smooth_axes[i] -= max_step;
                else smooth_axes[i] = js.axes[i];
                js.axes[i] = smooth_axes[i];
            }
            prev_ms = now;
        }

        /* 2 — map axes to control values */
        mavlink_packer_map(&js, &ctrl);

        /* 3 — apply safety (may override ctrl with neutral) */
        safety_update(&js, &ctrl);

        /* 4 — pack & send MANUAL_CONTROL */
        uint16_t len = mavlink_packer_pack_manual_control(
                            &ctrl, buf, sizeof(buf));
        if (len > 0) {
            if (transport_send(&transport, buf, len) > 0)
                packets_sent++;
        }

        /* 5 — heartbeat at 1 Hz */
        if (now - last_heartbeat >= HEARTBEAT_INTERVAL_MS) {
            len = mavlink_packer_pack_heartbeat(buf, sizeof(buf));
            if (len > 0)
                transport_send(&transport, buf, len);
            last_heartbeat = now;
        }

        /* 6 — receive telemetry from RPi */
        {
            uint8_t rxbuf[512];
            int rxn = transport_recv(&transport, rxbuf, sizeof(rxbuf));
            if (rxn > 0)
                telemetry_rx_feed(&telem_parser, &telem, rxbuf, rxn);
            telemetry_rx_tick(&telem, now);
        }

        /* 7 — Hz estimator */
        hz_frames++;
        if (now - hz_start >= 1000) {
            loop_hz   = (float)hz_frames * 1000.0f / (float)(now - hz_start);
            hz_start  = now;
            hz_frames = 0;
        }

        /* 8 — display */
        if (gui_mode) {
            gui_frame_t gf;
            memset(&gf, 0, sizeof(gf));

            memcpy(gf.js_axes, js.axes, sizeof(gf.js_axes));
            gf.js_num_axes    = js.num_axes;
            gf.js_buttons     = js.buttons;
            gf.js_num_buttons = js.num_buttons;
            gf.js_connected   = js.connected;

            gf.ctrl_x       = ctrl.x;
            gf.ctrl_y       = ctrl.y;
            gf.ctrl_z       = ctrl.z;
            gf.ctrl_r       = ctrl.r;
            gf.ctrl_buttons = ctrl.buttons;

            gf.failsafe       = safety_is_failsafe();
            gf.rov_connected  = telem.connected;
            gf.rov_armed      = telem.armed;
            gf.packets_sent   = packets_sent;
            gf.loop_hz        = loop_hz;
            gf.transport_str  = transport_str;

            /* Telemetry from RPi */
            gf.battery_voltage  = telem.battery_voltage;
            gf.battery_current  = telem.battery_current;
            gf.battery_percent  = telem.battery_percent;
            gf.water_temp_c     = telem.water_temp_c;
            gf.internal_temp_c  = telem.internal_temp_c;
            gf.depth_m          = telem.depth_m;
            gf.imu_roll         = telem.roll_deg;
            gf.imu_pitch        = telem.pitch_deg;
            gf.imu_yaw          = telem.yaw_deg;

            if (!gui_render(&gf))
                g_running = 0;
        } else {
            /* CLI status line */
            if (now - last_status >= STATUS_PRINT_INTERVAL_MS) {
                print_status(&js, &ctrl, safety_is_failsafe());
                last_status = now;
            }
            /* drain events in CLI mode so the queue doesn't grow */
            SDL_Event ev;
            while (SDL_PollEvent(&ev))
                ;
        }

        /* 8 — rate-limit */
        usleep(LOOP_PERIOD_MS * 1000);
    }

    /* ---- shutdown ---- */
    printf("\n\n[main] Shutting down — sending final neutral...\n");

    memset(&ctrl, 0, sizeof(ctrl));
    uint16_t len = mavlink_packer_pack_manual_control(
                        &ctrl, buf, sizeof(buf));
    if (len > 0)
        transport_send(&transport, buf, len);

    transport_close(&transport);
    joystick_close();
    if (gui_mode) gui_shutdown();
    SDL_Quit();

    printf("[main] Done.\n");
    return 0;
}
