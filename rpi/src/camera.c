/*
 * camera.c — RPi 5 camera streaming via rpicam-vid + GStreamer
 *
 * Launches rpicam-vid (works on RPi 5) piped into gst-launch-1.0
 * to produce an H264 RTP stream over UDP to the Ground Station PC.
 *
 * Pipeline:
 *   rpicam-vid → stdout (raw H264) → gst-launch-1.0 fdsrc
 *     → h264parse → rtph264pay → udpsink
 */

#include "camera.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
        /* Child process — exec rpicam-vid piped to gst-launch via sh -c */
        char cmd[512];
        snprintf(cmd, sizeof(cmd),
            "rpicam-vid -t 0 -n --width %d --height %d --framerate %d "
            "--codec h264 --inline --bitrate 2000000 -o - | "
            "gst-launch-1.0 fdsrc ! h264parse ! "
            "rtph264pay config-interval=1 pt=96 ! "
            "udpsink host=%s port=%d",
            width, height, fps, dest_ip, dest_port);

        execlp("sh", "sh", "-c", cmd, NULL);

        perror("[camera] execlp sh failed");
        _exit(1);
    }

    printf("[camera] Started rpicam-vid pipeline PID %d → %s:%d (%dx%d@%dfps)\n",
           pid, dest_ip, dest_port, width, height, fps);
    return pid;
}

void camera_stop(pid_t pid)
{
    if (pid <= 0) return;
    printf("[camera] Stopping PID %d\n", pid);
    /* Kill the whole process group (rpicam-vid + gst-launch) */
    kill(-pid, SIGTERM);
    kill(pid, SIGTERM);
    usleep(200000);
    /* Reap zombie */
    int status;
    waitpid(pid, &status, WNOHANG);
}

