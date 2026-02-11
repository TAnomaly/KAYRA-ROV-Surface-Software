/*
 * mavlink_parser.h — MAVLink v2 frame parser & telemetry sender
 *
 * Parses incoming MAVLink v2 frames from raw UDP bytes.
 * Also provides functions to pack and send telemetry back to GCS.
 */

#ifndef MAVLINK_PARSER_H
#define MAVLINK_PARSER_H

#include <stdint.h>
#include <stdbool.h>

/* ── MAVLink v2 constants ── */
#define MAVLINK_STX          0xFD
#define MAVLINK_HEADER_LEN   10
#define MAVLINK_CRC_LEN      2
#define MAVLINK_MAX_PAYLOAD  255
#define MAVLINK_MAX_FRAME    (MAVLINK_HEADER_LEN + MAVLINK_MAX_PAYLOAD + MAVLINK_CRC_LEN)

/* Message IDs */
#define MSG_ID_HEARTBEAT        0
#define MSG_ID_MANUAL_CONTROL   69

/* CRC extras */
#define CRC_EXTRA_HEARTBEAT        50
#define CRC_EXTRA_MANUAL_CONTROL   243

/* ── Parsed MANUAL_CONTROL ── */
typedef struct {
    int16_t  x;           /* surge  [-1000, +1000] */
    int16_t  y;           /* sway   [-1000, +1000] */
    int16_t  z;           /* heave  [-1000, +1000] */
    int16_t  r;           /* yaw    [-1000, +1000] */
    uint16_t buttons;
    uint8_t  target;
    bool     valid;       /* true if successfully parsed */
} manual_control_msg_t;

/* ── Parser state machine ── */
typedef enum {
    PARSE_IDLE,
    PARSE_GOT_STX,
    PARSE_GOT_HEADER,
    PARSE_GOT_PAYLOAD,
} parse_state_t;

typedef struct {
    parse_state_t state;
    uint8_t  buf[MAVLINK_MAX_FRAME];
    int      idx;
    uint8_t  payload_len;
    uint32_t msg_id;
} mavlink_parser_t;

/* Initialise parser */
void mavlink_parser_init(mavlink_parser_t *p);

/* Feed one byte; returns true when a complete valid frame is ready.
 * After returning true, call mavlink_parser_get_manual_control(). */
bool mavlink_parser_feed(mavlink_parser_t *p, uint8_t byte);

/* Extract MANUAL_CONTROL from the last parsed frame.
 * Returns false if the last frame was not MANUAL_CONTROL. */
bool mavlink_parser_get_manual_control(const mavlink_parser_t *p,
                                       manual_control_msg_t *out);

/* ── Telemetry packing (vehicle → GCS) ── */

/* Pack a HEARTBEAT frame.  Returns frame length, or -1. */
int mavlink_pack_heartbeat(uint8_t *buf, int bufsize, uint8_t *seq);

/* Pack an ATTITUDE frame (msg-id 30).  roll/pitch/yaw in RADIANS. */
int mavlink_pack_attitude(uint8_t *buf, int bufsize, uint8_t *seq,
                          uint32_t time_boot_ms,
                          float roll, float pitch, float yaw,
                          float rollspeed, float pitchspeed, float yawspeed);

/* Pack a SYS_STATUS frame (msg-id 1).
 *   voltage_mv  — battery voltage in milliVolts
 *   current_ca  — battery current in centi-Amps (10 mA)
 *   remaining   — battery remaining 0..100 %                     */
int mavlink_pack_sys_status(uint8_t *buf, int bufsize, uint8_t *seq,
                            uint16_t voltage_mv, int16_t current_ca,
                            int8_t remaining);

/* Pack a SCALED_PRESSURE frame (msg-id 29).
 *   press_abs   — absolute pressure in hPa
 *   press_diff  — differential pressure in hPa (depth)
 *   temperature — centi-degrees Celsius                           */
int mavlink_pack_scaled_pressure(uint8_t *buf, int bufsize, uint8_t *seq,
                                 uint32_t time_boot_ms,
                                 float press_abs, float press_diff,
                                 int16_t temperature);

#endif /* MAVLINK_PARSER_H */
