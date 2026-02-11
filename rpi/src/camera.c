/*
 * camera.c — ArduCam H264 streaming via GStreamer child process
 *
 * Launches a GStreamer pipeline that:
 *   libcamerasrc → H264 encode → RTP → UDP to GCS PC
 *
 * Runs as a forked child process so it doesn't block the main loop.
 */

#include "camera.h"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>

pid_t camera_start(const char *dest_ip, int dest_port,
                   int width, int height, int fps)
{
    pid_t pid = fork();

    if (pid < 0) {
        perror("[camera] fork");
        return -1;
    }

    if (pid == 0) {
        /* Child process — exec gst-launch-1.0 */
        char caps[128];
        snprintf(caps, sizeof(caps),
                 "video/x-raw,width=%d,height=%d,framerate=%d/1",
                 width, height, fps);

        char sink[128];
        snprintf(sink, sizeof(sink),
                 "host=%s port=%d", dest_ip, dest_port);

        /*
         * Pipeline:
         *   libcamerasrc → videoconvert → v4l2h264enc (HW encode on RPi 5)
         *   → h264parse → rtph264pay → udpsink
         *
         * If v4l2h264enc is not available, falls back to x264enc.
         */
        execlp("gst-launch-1.0", "gst-launch-1.0",
               "libcamerasrc", "!",
               caps, "!",
               "videoconvert", "!",
               "v4l2h264enc", "extra-controls=encode,video_bitrate=2000000", "!",
               "video/x-h264,level=(string)4", "!",
               "h264parse", "!",
               "rtph264pay", "config-interval=1", "pt=96", "!",
               "udpsink", sink,
               NULL);

        /* If v4l2h264enc failed, try software encoder */
        execlp("gst-launch-1.0", "gst-launch-1.0",
               "libcamerasrc", "!",
               caps, "!",
               "videoconvert", "!",
               "x264enc", "tune=zerolatency", "bitrate=2000",
               "speed-preset=ultrafast", "!",
               "rtph264pay", "config-interval=1", "pt=96", "!",
               "udpsink", sink,
               NULL);

        perror("[camera] execlp gst-launch-1.0 failed");
        _exit(1);
    }

    printf("[camera] Started pipeline PID %d → %s:%d (%dx%d@%dfps)\n",
           pid, dest_ip, dest_port, width, height, fps);
    return pid;
}

void camera_stop(pid_t pid)
{
    if (pid <= 0) return;
    printf("[camera] Stopping PID %d\n", pid);
    kill(pid, SIGTERM);
    usleep(200000);
    /* Reap zombie */
    int status;
    waitpid(pid, &status, WNOHANG);
}
