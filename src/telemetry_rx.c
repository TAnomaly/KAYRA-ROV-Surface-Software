/*
 * telemetry_rx.c — PC-side MAVLink v2 parser for incoming ROV telemetry
 *
 * Re-uses the same X.25 CRC and byte-by-byte state machine that the
 * RPi side uses, but extracts different message types:
 *   HEARTBEAT        (0)   → connection liveness
 *   SYS_STATUS       (1)   → battery voltage / current / remaining
 *   SCALED_PRESSURE  (29)  → depth (from press_diff) + water temperature
 *   ATTITUDE         (30)  → roll, pitch, yaw (radians → degrees)
 */

#include "telemetry_rx.h"
#include <string.h>
#include <math.h>

/* ── MAVLink v2 constants ── */
#define MAVLINK_STX           0xFD
#define MAVLINK_HEADER_LEN    10
#define MAVLINK_CRC_LEN       2
#define MAVLINK_MAX_PAYLOAD   255
#define MAVLINK_MAX_FRAME     (MAVLINK_HEADER_LEN + MAVLINK_MAX_PAYLOAD + MAVLINK_CRC_LEN)

/* Message IDs we care about */
#define MSG_HEARTBEAT         0
#define MSG_SYS_STATUS        1
#define MSG_SCALED_PRESSURE   29
#define MSG_ATTITUDE          30

/* CRC extras (from MAVLink XML) */
#define CRC_EXTRA_HEARTBEAT        50
#define CRC_EXTRA_SYS_STATUS       124
#define CRC_EXTRA_SCALED_PRESSURE  115
#define CRC_EXTRA_ATTITUDE         39

/* Parser states */
enum { IDLE = 0, GOT_STX, GOT_HEADER, GOT_PAYLOAD };

/* ── X.25 CRC ── */

static uint16_t crc_acc(uint8_t data, uint16_t crc)
{
    uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8)
         ^ ((uint16_t)tmp << 8)
         ^ ((uint16_t)tmp << 3)
         ^ ((uint16_t)tmp >> 4);
}

static uint16_t crc_calc(const uint8_t *buf, int len, uint8_t extra)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++)
        crc = crc_acc(buf[i], crc);
    crc = crc_acc(extra, crc);
    return crc;
}

static uint8_t crc_extra_for(uint32_t id)
{
    switch (id) {
    case MSG_HEARTBEAT:        return CRC_EXTRA_HEARTBEAT;
    case MSG_SYS_STATUS:       return CRC_EXTRA_SYS_STATUS;
    case MSG_SCALED_PRESSURE:  return CRC_EXTRA_SCALED_PRESSURE;
    case MSG_ATTITUDE:         return CRC_EXTRA_ATTITUDE;
    default:                   return 0;
    }
}

/* ── Little-endian helpers ── */

static float le_float(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);
    return f;
}

static uint16_t le_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)le_u16(p);
}

/* ── Time helper (same as safety.c) ── */

#include <time.h>
static uint64_t time_ms_local(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* ── Decode a parsed frame into telemetry state ── */

static void decode_frame(telemetry_parser_t *p, telemetry_state_t *ts)
{
    const uint8_t *pl = &p->buf[MAVLINK_HEADER_LEN];

    switch (p->msg_id) {

    case MSG_HEARTBEAT:
        ts->last_heartbeat_ms = time_ms_local();
        ts->connected = true;
        break;

    case MSG_SYS_STATUS:
        /* payload layout (standard MAVLink wire order):
         *   [0..3]   sensors_present  uint32
         *   [4..7]   sensors_enabled  uint32
         *   [8..11]  sensors_health   uint32
         *   [12..13] load             uint16
         *   [14..15] voltage_battery  uint16  (mV)
         *   [16..17] current_battery  int16   (cA = 10 mA)
         *   [18]     battery_remaining int8   (0..100 %)
         */
        if (p->payload_len >= 19) {
            ts->battery_voltage = (float)le_u16(&pl[14]) / 1000.0f;
            ts->battery_current = (float)le_i16(&pl[16]) / 100.0f;
            ts->battery_percent = (float)(int8_t)pl[18];
            if (ts->battery_percent < 0.0f) ts->battery_percent = 0.0f;
        }
        break;

    case MSG_ATTITUDE:
        /* [0..3]  time_boot_ms  uint32
         * [4..7]  roll          float (rad)
         * [8..11] pitch         float (rad)
         * [12..15] yaw          float (rad)  */
        if (p->payload_len >= 16) {
            ts->roll_deg  = le_float(&pl[4])  * (180.0f / (float)M_PI);
            ts->pitch_deg = le_float(&pl[8])  * (180.0f / (float)M_PI);
            float yaw_rad = le_float(&pl[12]);
            ts->yaw_deg = yaw_rad * (180.0f / (float)M_PI);
            if (ts->yaw_deg < 0.0f) ts->yaw_deg += 360.0f;
        }
        break;

    case MSG_SCALED_PRESSURE:
        /* [0..3]  time_boot_ms  uint32
         * [4..7]  press_abs     float (hPa)
         * [8..11] press_diff    float (hPa)  → depth
         * [12..13] temperature  int16 (centi-degrees)  */
        if (p->payload_len >= 14) {
            float press_diff = le_float(&pl[8]);
            /* depth ≈ press_diff_hPa / 100  (1 hPa ≈ 1 cm H₂O) */
            ts->depth_m = press_diff / 100.0f;
            if (ts->depth_m < 0.0f) ts->depth_m = 0.0f;
            ts->water_temp_c = (float)le_i16(&pl[12]) / 100.0f;
        }
        break;
    }

    ts->packets_received++;
}

/* ── Public API ── */

void telemetry_rx_init(telemetry_parser_t *p, telemetry_state_t *ts)
{
    memset(p, 0, sizeof(*p));
    memset(ts, 0, sizeof(*ts));
    p->state = IDLE;
}

void telemetry_rx_feed(telemetry_parser_t *p, telemetry_state_t *ts,
                       const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        uint8_t byte = data[i];

        switch (p->state) {

        case IDLE:
            if (byte == MAVLINK_STX) {
                p->buf[0] = byte;
                p->idx = 1;
                p->state = GOT_STX;
            }
            break;

        case GOT_STX:
            p->buf[p->idx++] = byte;
            if (p->idx == MAVLINK_HEADER_LEN) {
                p->payload_len = p->buf[1];
                p->msg_id = (uint32_t)p->buf[7]
                          | ((uint32_t)p->buf[8] << 8)
                          | ((uint32_t)p->buf[9] << 16);
                if (MAVLINK_HEADER_LEN + (int)p->payload_len + MAVLINK_CRC_LEN
                    > MAVLINK_MAX_FRAME) {
                    p->state = IDLE;
                    break;
                }
                p->state = GOT_HEADER;
            }
            break;

        case GOT_HEADER:
            if (p->idx >= MAVLINK_MAX_FRAME) { p->state = IDLE; break; }
            p->buf[p->idx++] = byte;
            if (p->idx == MAVLINK_HEADER_LEN + p->payload_len)
                p->state = GOT_PAYLOAD;
            break;

        case GOT_PAYLOAD:
            if (p->idx >= MAVLINK_MAX_FRAME) { p->state = IDLE; break; }
            p->buf[p->idx++] = byte;
            if (p->idx == MAVLINK_HEADER_LEN + p->payload_len + MAVLINK_CRC_LEN) {
                /* Verify CRC */
                uint8_t extra = crc_extra_for(p->msg_id);
                uint16_t expected = crc_calc(
                    &p->buf[1],
                    MAVLINK_HEADER_LEN - 1 + p->payload_len,
                    extra);

                int off = MAVLINK_HEADER_LEN + p->payload_len;
                uint16_t received = (uint16_t)p->buf[off]
                                  | ((uint16_t)p->buf[off + 1] << 8);

                if (expected == received)
                    decode_frame(p, ts);

                p->state = IDLE;
            }
            break;
        }
    }
}

void telemetry_rx_tick(telemetry_state_t *ts, uint64_t now_ms)
{
    /* Mark disconnected if no heartbeat for 3 seconds */
    if (ts->last_heartbeat_ms > 0 &&
        (now_ms - ts->last_heartbeat_ms) > 3000) {
        ts->connected = false;
    }
}
