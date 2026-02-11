/*
 * telemetry_rx.h — Receive & parse MAVLink telemetry from the ROV
 *
 * Parses incoming MAVLink v2 frames (HEARTBEAT, SYS_STATUS, ATTITUDE,
 * SCALED_PRESSURE) and stores the latest values.
 */

#ifndef TELEMETRY_RX_H
#define TELEMETRY_RX_H

#include <stdint.h>
#include <stdbool.h>

/* ── Telemetry snapshot (all fields updated as new packets arrive) ── */
typedef struct {
    /* Connection */
    bool     connected;           /* true if heartbeat within last 3 s  */
    uint64_t last_heartbeat_ms;   /* timestamp of last HEARTBEAT        */
    uint64_t packets_received;    /* total parsed frames                */

    /* Battery  (from SYS_STATUS, msg-id 1) */
    float    battery_voltage;     /* Volts                              */
    float    battery_current;     /* Amps                               */
    float    battery_percent;     /* 0..100                             */

    /* Orientation  (from ATTITUDE, msg-id 30) */
    float    roll_deg;            /* degrees, + = right wing down       */
    float    pitch_deg;           /* degrees, + = nose up               */
    float    yaw_deg;             /* degrees, 0..360 from North         */

    /* Environment  (from SCALED_PRESSURE, msg-id 29) */
    float    depth_m;             /* metres (computed from press_diff)   */
    float    water_temp_c;        /* Celsius                            */

    /* Internal temperature (from SCALED_IMU2 or custom, msg-id 116) */
    float    internal_temp_c;
} telemetry_state_t;

/* ── Parser state machine (same structure as RPi side) ── */
typedef struct {
    int      state;
    uint8_t  buf[277];            /* MAVLINK_MAX_FRAME                  */
    int      idx;
    uint8_t  payload_len;
    uint32_t msg_id;
} telemetry_parser_t;

/* Initialise parser and zero out the telemetry state. */
void telemetry_rx_init(telemetry_parser_t *p, telemetry_state_t *ts);

/* Feed raw bytes.  Call after each transport_recv().
 * Internally parses MAVLink frames and updates *ts.          */
void telemetry_rx_feed(telemetry_parser_t *p, telemetry_state_t *ts,
                       const uint8_t *data, int len);

/* Call once per main-loop iteration to update derived fields
 * (e.g. connected flag based on heartbeat timeout).            */
void telemetry_rx_tick(telemetry_state_t *ts, uint64_t now_ms);

#endif /* TELEMETRY_RX_H */
