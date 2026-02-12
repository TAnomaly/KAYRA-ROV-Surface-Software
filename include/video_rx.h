/*
 * video_rx.h — GStreamer H264 UDP stream receiver for Surface Control
 *
 * Receives an RTP/H264 stream over UDP, decodes it to RGB frames,
 * and makes the latest frame available to the main loop via
 * video_rx_get_frame().
 */

#ifndef VIDEO_RX_H
#define VIDEO_RX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start receiving H264 stream on the given UDP port.
 * Returns 0 on success, -1 on error. */
int video_rx_start(int udp_port);

/* Get the latest decoded frame.
 * Sets *rgb to the internal buffer (valid until next call), *w and *h
 * to the frame dimensions.
 * Returns true if a frame is available, false otherwise.
 * The returned pointer is valid until the next call to video_rx_get_frame(). */
bool video_rx_get_frame(const uint8_t **rgb, int *w, int *h);

/* Stop the receiver and free all resources. */
void video_rx_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* VIDEO_RX_H */
