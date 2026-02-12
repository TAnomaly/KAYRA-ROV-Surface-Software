/*
 * video_rx.c — GStreamer H264 UDP stream receiver
 *
 * Pipeline:
 *   udpsrc port=N → application/x-rtp,encoding-name=H264,payload=96
 *   → rtph264depay → h264parse → avdec_h264 → videoconvert
 *   → video/x-raw,format=RGB → appsink
 *
 * Uses double buffering for thread safety:
 *   - Back buffer:  GStreamer thread writes decoded frames here
 *   - Front buffer: Main thread reads from here
 *   - On new frame: swap the buffers under mutex
 */

#include "video_rx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

/* ------------------------------------------------------------------ */

static GstElement     *s_pipeline = NULL;
static pthread_mutex_t s_mutex    = PTHREAD_MUTEX_INITIALIZER;

/* Double-buffered frame storage */
static uint8_t *s_buf[2]      = { NULL, NULL };
static int      s_buf_alloc[2] = { 0, 0 };
static int      s_write_idx    = 0;     /* GStreamer writes to this index  */
static int      s_frame_w      = 0;
static int      s_frame_h      = 0;
static bool     s_has_frame    = false; /* at least one frame decoded      */

/* ------------------------------------------------------------------ */

/*
 * Called by GStreamer on the streaming thread whenever a new decoded
 * frame arrives at the appsink.
 */
static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data)
{
    (void)user_data;

    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps   *caps   = gst_sample_get_caps(sample);
    if (!buffer || !caps) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    /* Extract width/height from caps */
    GstStructure *s = gst_caps_get_structure(caps, 0);
    int w = 0, h = 0;
    gst_structure_get_int(s, "width",  &w);
    gst_structure_get_int(s, "height", &h);
    if (w <= 0 || h <= 0) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    int need = w * h * 3;   /* RGB24 */

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    if ((int)map.size < need) {
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    /* Write to back buffer (no lock needed — only this thread writes) */
    int wi = s_write_idx;
    if (s_buf_alloc[wi] < need) {
        free(s_buf[wi]);
        s_buf[wi]       = (uint8_t *)malloc(need);
        s_buf_alloc[wi] = need;
    }

    if (s_buf[wi]) {
        memcpy(s_buf[wi], map.data, need);

        /* Swap: make the back buffer become the front buffer */
        pthread_mutex_lock(&s_mutex);
        s_write_idx = 1 - wi;   /* next write goes to the other buffer */
        s_frame_w   = w;
        s_frame_h   = h;
        s_has_frame = true;
        pthread_mutex_unlock(&s_mutex);
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    /* One-shot log on first frame */
    static bool first = true;
    if (first) {
        printf("[video_rx] First frame decoded: %dx%d\n", w, h);
        first = false;
    }

    return GST_FLOW_OK;
}

/* ------------------------------------------------------------------ */

int video_rx_start(int udp_port)
{
    /* Initialize GStreamer (safe to call multiple times) */
    gst_init(NULL, NULL);

    /*
     * Build pipeline string.
     * Using appsink so we get raw RGB frames in our callback.
     */
    char launch[512];
    snprintf(launch, sizeof(launch),
        "udpsrc port=%d"
        " ! application/x-rtp,encoding-name=H264,payload=96"
        " ! rtph264depay"
        " ! h264parse"
        " ! avdec_h264"
        " ! videoconvert"
        " ! video/x-raw,format=RGB"
        " ! appsink name=sink emit-signals=true sync=false"
        " max-buffers=2 drop=true",
        udp_port);

    GError *err = NULL;
    s_pipeline = gst_parse_launch(launch, &err);
    if (!s_pipeline) {
        fprintf(stderr, "[video_rx] Pipeline parse error: %s\n",
                err ? err->message : "unknown");
        if (err) g_error_free(err);
        return -1;
    }
    if (err) {
        fprintf(stderr, "[video_rx] Pipeline warning: %s\n", err->message);
        g_error_free(err);
    }

    /* Hook up the appsink callback */
    GstElement *sink = gst_bin_get_by_name(GST_BIN(s_pipeline), "sink");
    if (!sink) {
        fprintf(stderr, "[video_rx] Could not find appsink element\n");
        gst_object_unref(s_pipeline);
        s_pipeline = NULL;
        return -1;
    }

    GstAppSinkCallbacks cbs = { 0 };
    cbs.new_sample = on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(sink), &cbs, NULL, NULL);
    gst_object_unref(sink);

    /* Start the pipeline */
    GstStateChangeReturn ret = gst_element_set_state(s_pipeline,
                                                      GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "[video_rx] Failed to start pipeline\n");
        gst_object_unref(s_pipeline);
        s_pipeline = NULL;
        return -1;
    }

    printf("[video_rx] Receiving H264 stream on UDP port %d\n", udp_port);
    return 0;
}

/* ------------------------------------------------------------------ */

bool video_rx_get_frame(const uint8_t **rgb, int *w, int *h)
{
    pthread_mutex_lock(&s_mutex);

    if (!s_has_frame) {
        pthread_mutex_unlock(&s_mutex);
        return false;
    }

    /* Read from front buffer (the one GStreamer is NOT writing to) */
    int read_idx = 1 - s_write_idx;
    *rgb = s_buf[read_idx];
    *w   = s_frame_w;
    *h   = s_frame_h;

    pthread_mutex_unlock(&s_mutex);
    return (*rgb != NULL);
}

/* ------------------------------------------------------------------ */

void video_rx_stop(void)
{
    if (s_pipeline) {
        gst_element_set_state(s_pipeline, GST_STATE_NULL);
        gst_object_unref(s_pipeline);
        s_pipeline = NULL;
        printf("[video_rx] Pipeline stopped\n");
    }

    pthread_mutex_lock(&s_mutex);
    free(s_buf[0]); s_buf[0] = NULL; s_buf_alloc[0] = 0;
    free(s_buf[1]); s_buf[1] = NULL; s_buf_alloc[1] = 0;
    s_write_idx = 0;
    s_frame_w   = 0;
    s_frame_h   = 0;
    s_has_frame = false;
    pthread_mutex_unlock(&s_mutex);
}
