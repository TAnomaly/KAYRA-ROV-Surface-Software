/*
 * camera.h — ArduCam H264 stream via GStreamer on RPi 5
 *
 * Launches a GStreamer pipeline as a child process that captures from
 * libcamera and sends H264 RTP to the ground station PC.
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <sys/types.h>

/* Start the camera GStreamer pipeline.
 * Returns child PID on success, -1 on error. */
pid_t camera_start(const char *dest_ip, int dest_port,
                   int width, int height, int fps);

/* Stop the camera pipeline (kills child process). */
void camera_stop(pid_t pid);

#endif /* CAMERA_H */
