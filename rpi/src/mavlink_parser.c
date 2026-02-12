/*
 * mavlink_parser.c — MAVLink v2 byte-by-byte parser + telemetry packer
 */

#include "mavlink_parser.h"
#include "config.h"
#include <string.h>

/* ── X.25 CRC (same as PC side) ── */

static uint16_t crc_accumulate(uint8_t data, uint16_t crc)
{
    uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8)
         ^ ((uint16_t)tmp << 8)
         ^ ((uint16_t)tmp << 3)
         ^ ((uint16_t)tmp >> 4);
}

static uint16_t crc_calculate(const uint8_t *buf, int len, uint8_t crc_extra)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++)
        crc = crc_accumulate(buf[i], crc);
    crc = crc_accumulate(crc_extra, crc);
    return crc;
}

/* ── CRC extra lookup ── */

/* CRC extras for all message types we handle */
#define CRC_EXTRA_SYS_STATUS       124
#define CRC_EXTRA_SCALED_PRESSURE  115
#define CRC_EXTRA_ATTITUDE         39

static uint8_t crc_extra_for_msgid(uint32_t id)
{
    switch (id) {
    case MSG_ID_HEARTBEAT:       return CRC_EXTRA_HEARTBEAT;
    case MSG_ID_MANUAL_CONTROL:  return CRC_EXTRA_MANUAL_CONTROL;
    case 1:                      return CRC_EXTRA_SYS_STATUS;
    case 29:                     return CRC_EXTRA_SCALED_PRESSURE;
    case 30:                     return CRC_EXTRA_ATTITUDE;
    default:                     return 0;
    }
}

/* ── Parser ── */

void mavlink_parser_init(mavlink_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = PARSE_IDLE;
}

bool mavlink_parser_feed(mavlink_parser_t *p, uint8_t byte)
{
    switch (p->state) {

    case PARSE_IDLE:
        if (byte == MAVLINK_STX) {
            p->buf[0] = byte;
            p->idx = 1;
            p->state = PARSE_GOT_STX;
        }
        return false;

    case PARSE_GOT_STX:
        p->buf[p->idx++] = byte;
        if (p->idx == MAVLINK_HEADER_LEN) {
            p->payload_len = p->buf[1];
            p->msg_id = (uint32_t)p->buf[7]
                      | ((uint32_t)p->buf[8] << 8)
                      | ((uint32_t)p->buf[9] << 16);
            /* Guard: ensure total frame fits in buf[] */
            if (MAVLINK_HEADER_LEN + (int)p->payload_len + MAVLINK_CRC_LEN
                > MAVLINK_MAX_FRAME) {
                p->state = PARSE_IDLE;
                return false;
            }
            p->state = PARSE_GOT_HEADER;
        }
        return false;

    case PARSE_GOT_HEADER:
        if (p->idx >= MAVLINK_MAX_FRAME) { p->state = PARSE_IDLE; return false; }
        p->buf[p->idx++] = byte;
        if (p->idx == MAVLINK_HEADER_LEN + p->payload_len) {
            p->state = PARSE_GOT_PAYLOAD;
        }
        return false;

    case PARSE_GOT_PAYLOAD:
        if (p->idx >= MAVLINK_MAX_FRAME) { p->state = PARSE_IDLE; return false; }
        p->buf[p->idx++] = byte;
        if (p->idx == MAVLINK_HEADER_LEN + p->payload_len + MAVLINK_CRC_LEN) {
            /* Verify CRC */
            uint8_t extra = crc_extra_for_msgid(p->msg_id);
            /* CRC covers bytes 1..9 (header minus STX) + payload */
            uint16_t expected = crc_calculate(
                &p->buf[1],
                MAVLINK_HEADER_LEN - 1 + p->payload_len,
                extra);

            int crc_off = MAVLINK_HEADER_LEN + p->payload_len;
            uint16_t received = (uint16_t)p->buf[crc_off]
                              | ((uint16_t)p->buf[crc_off + 1] << 8);

            p->state = PARSE_IDLE;
            return (expected == received);
        }
        return false;
    }

    p->state = PARSE_IDLE;
    return false;
}

bool mavlink_parser_get_manual_control(const mavlink_parser_t *p,
                                       manual_control_msg_t *out)
{
    if (p->msg_id != MSG_ID_MANUAL_CONTROL)
        return false;

    const uint8_t *pl = &p->buf[MAVLINK_HEADER_LEN];

    out->x       = (int16_t)((uint16_t)pl[0] | ((uint16_t)pl[1] << 8));
    out->y       = (int16_t)((uint16_t)pl[2] | ((uint16_t)pl[3] << 8));
    out->z       = (int16_t)((uint16_t)pl[4] | ((uint16_t)pl[5] << 8));
    out->r       = (int16_t)((uint16_t)pl[6] | ((uint16_t)pl[7] << 8));
    out->buttons = (uint16_t)pl[8] | ((uint16_t)pl[9] << 8);
    out->target  = pl[10];
    out->valid   = true;

    return true;
}

/* ── Helper: fill MAVLink v2 header + compute CRC, return total length ── */

static int pack_frame(uint8_t *buf, int bufsize, uint8_t *seq,
                      uint32_t msgid, uint8_t crc_extra,
                      const uint8_t *payload, uint8_t payload_len)
{
    int total = MAVLINK_HEADER_LEN + payload_len + MAVLINK_CRC_LEN;
    if (bufsize < total) return -1;

    buf[0] = MAVLINK_STX;
    buf[1] = payload_len;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = (*seq)++;
    buf[5] = VEHICLE_SYSID;
    buf[6] = VEHICLE_COMPID;
    buf[7] = (uint8_t)(msgid & 0xFF);
    buf[8] = (uint8_t)((msgid >> 8)  & 0xFF);
    buf[9] = (uint8_t)((msgid >> 16) & 0xFF);

    memcpy(&buf[MAVLINK_HEADER_LEN], payload, payload_len);

    uint16_t crc = crc_calculate(&buf[1],
        MAVLINK_HEADER_LEN - 1 + payload_len, crc_extra);

    int off = MAVLINK_HEADER_LEN + payload_len;
    buf[off]     = (uint8_t)(crc & 0xFF);
    buf[off + 1] = (uint8_t)((crc >> 8) & 0xFF);

    return total;
}

/* ── Little-endian store helpers ── */

static void put_u16(uint8_t *p, uint16_t v)  { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put_i16(uint8_t *p, int16_t v)   { put_u16(p, (uint16_t)v); }
static void put_u32(uint8_t *p, uint32_t v)  { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8);
                                                p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }
static void put_float(uint8_t *p, float v)   { memcpy(p, &v, 4); }

/* ── Telemetry: pack HEARTBEAT (msg-id 0, 9 bytes) ── */

int mavlink_pack_heartbeat(uint8_t *buf, int bufsize, uint8_t *seq, int armed)
{
    uint8_t pl[9];
    memset(pl, 0, sizeof(pl));
    put_u32(&pl[0], 0);     /* custom_mode = 0        */
    pl[4] = 2;               /* MAV_TYPE_SUBMARINE     */
    pl[5] = 8;               /* MAV_AUTOPILOT_INVALID  */
    /* base_mode: bit 7 = SAFETY_ARMED, bit 6 = MANUAL_INPUT */
    pl[6] = 64 | (armed ? 128 : 0);
    pl[7] = 4;               /* MAV_STATE_ACTIVE       */
    pl[8] = 3;               /* mavlink_version        */

    return pack_frame(buf, bufsize, seq, MSG_ID_HEARTBEAT,
                      CRC_EXTRA_HEARTBEAT, pl, 9);
}

/* ── Telemetry: pack ATTITUDE (msg-id 30, 28 bytes) ── */

int mavlink_pack_attitude(uint8_t *buf, int bufsize, uint8_t *seq,
                          uint32_t time_boot_ms,
                          float roll, float pitch, float yaw,
                          float rollspeed, float pitchspeed, float yawspeed)
{
    uint8_t pl[28];
    memset(pl, 0, sizeof(pl));
    put_u32 (&pl[0],  time_boot_ms);
    put_float(&pl[4],  roll);
    put_float(&pl[8],  pitch);
    put_float(&pl[12], yaw);
    put_float(&pl[16], rollspeed);
    put_float(&pl[20], pitchspeed);
    put_float(&pl[24], yawspeed);

    return pack_frame(buf, bufsize, seq, 30, CRC_EXTRA_ATTITUDE, pl, 28);
}

/* ── Telemetry: pack SYS_STATUS (msg-id 1, 31 bytes) ── */

int mavlink_pack_sys_status(uint8_t *buf, int bufsize, uint8_t *seq,
                            uint16_t voltage_mv, int16_t current_ca,
                            int8_t remaining)
{
    uint8_t pl[31];
    memset(pl, 0, sizeof(pl));
    /* sensors fields [0..13] stay 0 */
    put_u16(&pl[14], voltage_mv);
    put_i16(&pl[16], current_ca);
    pl[18] = (uint8_t)remaining;

    return pack_frame(buf, bufsize, seq, 1, CRC_EXTRA_SYS_STATUS, pl, 31);
}

/* ── Telemetry: pack SCALED_PRESSURE (msg-id 29, 14 bytes) ── */

int mavlink_pack_scaled_pressure(uint8_t *buf, int bufsize, uint8_t *seq,
                                 uint32_t time_boot_ms,
                                 float press_abs, float press_diff,
                                 int16_t temperature)
{
    uint8_t pl[14];
    memset(pl, 0, sizeof(pl));
    put_u32  (&pl[0],  time_boot_ms);
    put_float(&pl[4],  press_abs);
    put_float(&pl[8],  press_diff);
    put_i16  (&pl[12], temperature);

    return pack_frame(buf, bufsize, seq, 29, CRC_EXTRA_SCALED_PRESSURE, pl, 14);
}
