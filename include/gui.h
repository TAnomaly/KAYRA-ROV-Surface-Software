/*
 * gui.h — Dear ImGui visual dashboard for Surface Control GCS
 *
 * C-compatible interface.  Implementation in gui.cpp (C++).
 */

#ifndef GUI_H
#define GUI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Snapshot of system state fed to the GUI each frame. */
typedef struct {
    /* Raw joystick */
    float    js_axes[8];
    int      js_num_axes;
    uint16_t js_buttons;
    int      js_num_buttons;
    bool     js_connected;

    /* Commanded control values [-1000, +1000] */
    int16_t  ctrl_x;        /* surge  */
    int16_t  ctrl_y;        /* sway   */
    int16_t  ctrl_z;        /* heave  */
    int16_t  ctrl_r;        /* yaw    */
    uint16_t ctrl_buttons;

    /* System status */
    bool     failsafe;
    bool     rov_connected;     /* heartbeat received within 3s  */
    bool     rov_armed;         /* ROV motors armed              */
    uint64_t packets_sent;
    float    loop_hz;
    const char *transport_str;  /* e.g. "UDP 192.168.2.1:14550" */

    /* Camera frame (optional — leave zeroed / NULL when not available).
     * When a feed is connected, set camera_rgb to a pointer to RGB24
     * pixel data (3 bytes per pixel, row-major) and camera_w / camera_h
     * to the frame dimensions.  The GUI will upload it as a GL texture. */
    const uint8_t *camera_rgb;
    int            camera_w;
    int            camera_h;

    /* Battery (set to 0 when unknown) */
    float    battery_voltage;   /* Volts  (e.g. 12.6)   */
    float    battery_current;   /* Amps   (e.g. 3.2)    */
    float    battery_percent;   /* 0..100                */

    /* Telemetry sensors */
    float    water_temp_c;      /* water temperature °C  */
    float    internal_temp_c;   /* electronics temp °C   */
    float    depth_m;           /* depth in metres       */

    /* IMU orientation for 3-D model (degrees) */
    float    imu_roll;          /* degrees, + = right    */
    float    imu_pitch;         /* degrees, + = nose up  */
    float    imu_yaw;           /* degrees, 0 = North    */
} gui_frame_t;

/*  Create window + OpenGL context + ImGui.  Returns 0 on success. */
int  gui_init(int width, int height);

/*  Process events, render one frame.
 *  Returns true to keep running, false when the user closes the window. */
bool gui_render(const gui_frame_t *state);

/*  Destroy window and free resources.  */
void gui_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif /* GUI_H */
