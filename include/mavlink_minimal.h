/*
 * mavlink_minimal.h — Self-contained MAVLink v2 implementation
 *
 * Provides just enough of the MAVLink v2 wire protocol to pack and
 * serialize HEARTBEAT and MANUAL_CONTROL messages.  Drop-in compatible
 * with the real MAVLink C headers — if you later generate the full
 * library, simply replace this file.
 *
 * Wire format reference:
 *   https://mavlink.io/en/guide/serialization.html
 *
 * CRC_EXTRA values come from the MAVLink message XML definitions.
 */

#ifndef MAVLINK_MINIMAL_H
#define MAVLINK_MINIMAL_H

#include <stdint.h>
#include <string.h>

/* ================================================================== */
/*  Constants                                                         */
/* ================================================================== */

#define MAVLINK_STX_V2              0xFD
#define MAVLINK_MAX_PAYLOAD_LEN     255
#define MAVLINK_NUM_HEADER_BYTES    10
#define MAVLINK_NUM_CHECKSUM_BYTES  2
#define MAVLINK_MAX_PACKET_LEN      (MAVLINK_NUM_HEADER_BYTES + \
                                     MAVLINK_MAX_PAYLOAD_LEN  + \
                                     MAVLINK_NUM_CHECKSUM_BYTES)

/* Message IDs */
#define MAVLINK_MSG_ID_HEARTBEAT        0
#define MAVLINK_MSG_ID_MANUAL_CONTROL   69

/* CRC extras (seed appended to CRC for each message type) */
#define MAVLINK_MSG_HEARTBEAT_CRC           50
#define MAVLINK_MSG_MANUAL_CONTROL_CRC      243

/* Payload lengths */
#define MAVLINK_MSG_HEARTBEAT_LEN           9
#define MAVLINK_MSG_MANUAL_CONTROL_LEN      11

/* Useful MAV_TYPE / MAV_AUTOPILOT / MAV_STATE constants */
#define MAV_TYPE_GCS                        6
#define MAV_AUTOPILOT_INVALID               8
#define MAV_MODE_FLAG_MANUAL_INPUT_ENABLED  64
#define MAV_STATE_ACTIVE                    4

/* ================================================================== */
/*  Core types                                                        */
/* ================================================================== */

typedef struct {
    uint8_t  magic;
    uint8_t  len;               /* payload length           */
    uint8_t  incompat_flags;
    uint8_t  compat_flags;
    uint8_t  seq;
    uint8_t  sysid;
    uint8_t  compid;
    uint32_t msgid;             /* only lower 24 bits used  */
    uint8_t  payload[MAVLINK_MAX_PAYLOAD_LEN];
    uint16_t checksum;
} mavlink_message_t;

/* ================================================================== */
/*  X.25 CRC (ITU-T / CCITT)                                         */
/* ================================================================== */

static inline void mavlink_crc_init(uint16_t *crc)
{
    *crc = 0xFFFF;
}

static inline void mavlink_crc_accumulate(uint8_t data, uint16_t *crc)
{
    uint8_t tmp;
    tmp  = data ^ (uint8_t)(*crc & 0xFF);
    tmp ^= (tmp << 4);
    *crc = (*crc >> 8)
         ^ ((uint16_t)tmp << 8)
         ^ ((uint16_t)tmp << 3)
         ^ ((uint16_t)tmp >> 4);
}

/* ================================================================== */
/*  Frame finalization — compute CRC, fill header fields              */
/* ================================================================== */

/*
 * CRC covers: bytes 1-9 of header (everything except STX) + payload.
 * Then the message-specific CRC_EXTRA byte is accumulated.
 */
static inline void mavlink_finalize(mavlink_message_t *msg,
                                    uint8_t sysid, uint8_t compid,
                                    uint8_t payload_len, uint32_t msgid,
                                    uint8_t crc_extra, uint8_t *seq)
{
    msg->magic          = MAVLINK_STX_V2;
    msg->len            = payload_len;
    msg->incompat_flags = 0;
    msg->compat_flags   = 0;
    msg->seq            = (*seq)++;
    msg->sysid          = sysid;
    msg->compid         = compid;
    msg->msgid          = msgid;

    /* Build the 9 header bytes that enter the CRC */
    uint8_t hdr[9];
    hdr[0] = msg->len;
    hdr[1] = msg->incompat_flags;
    hdr[2] = msg->compat_flags;
    hdr[3] = msg->seq;
    hdr[4] = msg->sysid;
    hdr[5] = msg->compid;
    hdr[6] = (uint8_t)(msgid        & 0xFF);
    hdr[7] = (uint8_t)((msgid >> 8) & 0xFF);
    hdr[8] = (uint8_t)((msgid >> 16)& 0xFF);

    uint16_t crc;
    mavlink_crc_init(&crc);

    for (int i = 0; i < 9; i++)
        mavlink_crc_accumulate(hdr[i], &crc);

    for (int i = 0; i < payload_len; i++)
        mavlink_crc_accumulate(msg->payload[i], &crc);

    mavlink_crc_accumulate(crc_extra, &crc);

    msg->checksum = crc;
}

/* ================================================================== */
/*  Serialize a mavlink_message_t into a byte buffer for TX           */
/* ================================================================== */

static inline uint16_t mavlink_msg_to_send_buffer(uint8_t *buf,
                                                   const mavlink_message_t *msg)
{
    uint16_t i = 0;

    buf[i++] = MAVLINK_STX_V2;
    buf[i++] = msg->len;
    buf[i++] = msg->incompat_flags;
    buf[i++] = msg->compat_flags;
    buf[i++] = msg->seq;
    buf[i++] = msg->sysid;
    buf[i++] = msg->compid;
    buf[i++] = (uint8_t)(msg->msgid        & 0xFF);
    buf[i++] = (uint8_t)((msg->msgid >> 8)  & 0xFF);
    buf[i++] = (uint8_t)((msg->msgid >> 16) & 0xFF);

    memcpy(&buf[i], msg->payload, msg->len);
    i += msg->len;

    buf[i++] = (uint8_t)(msg->checksum       & 0xFF);
    buf[i++] = (uint8_t)((msg->checksum >> 8) & 0xFF);

    return i;   /* total frame length */
}

/* ================================================================== */
/*  HEARTBEAT  (msg-id 0, 9 bytes)                                   */
/*                                                                    */
/*  Wire order (fields sorted by type-width, descending):             */
/*    [0..3]  custom_mode     uint32_t                                */
/*    [4]     type            uint8_t                                 */
/*    [5]     autopilot       uint8_t                                 */
/*    [6]     base_mode       uint8_t                                 */
/*    [7]     system_status   uint8_t                                 */
/*    [8]     mavlink_version uint8_t  (always 3 for v2)              */
/* ================================================================== */

static inline void mavlink_msg_heartbeat_pack(
        mavlink_message_t *msg,
        uint8_t sysid, uint8_t compid,
        uint8_t type, uint8_t autopilot,
        uint8_t base_mode, uint32_t custom_mode,
        uint8_t system_status,
        uint8_t *seq)
{
    memset(msg->payload, 0, MAVLINK_MSG_HEARTBEAT_LEN);

    msg->payload[0] = (uint8_t)(custom_mode        & 0xFF);
    msg->payload[1] = (uint8_t)((custom_mode >> 8)  & 0xFF);
    msg->payload[2] = (uint8_t)((custom_mode >> 16) & 0xFF);
    msg->payload[3] = (uint8_t)((custom_mode >> 24) & 0xFF);
    msg->payload[4] = type;
    msg->payload[5] = autopilot;
    msg->payload[6] = base_mode;
    msg->payload[7] = system_status;
    msg->payload[8] = 3;   /* mavlink_version — "2.0" is reported as 3 */

    mavlink_finalize(msg, sysid, compid,
                     MAVLINK_MSG_HEARTBEAT_LEN,
                     MAVLINK_MSG_ID_HEARTBEAT,
                     MAVLINK_MSG_HEARTBEAT_CRC,
                     seq);
}

/* ================================================================== */
/*  MANUAL_CONTROL  (msg-id 69, 11 bytes)                            */
/*                                                                    */
/*  Wire order (fields sorted by type-width, descending):             */
/*    [0..1]  x        int16_t   surge                                */
/*    [2..3]  y        int16_t   sway                                 */
/*    [4..5]  z        int16_t   heave                                */
/*    [6..7]  r        int16_t   yaw                                  */
/*    [8..9]  buttons  uint16_t                                       */
/*    [10]    target   uint8_t                                        */
/* ================================================================== */

static inline void mavlink_msg_manual_control_pack(
        mavlink_message_t *msg,
        uint8_t sysid, uint8_t compid,
        uint8_t target,
        int16_t x, int16_t y, int16_t z, int16_t r,
        uint16_t buttons,
        uint8_t *seq)
{
    memset(msg->payload, 0, MAVLINK_MSG_MANUAL_CONTROL_LEN);

    /* All multi-byte fields are little-endian on the wire */
    msg->payload[0]  = (uint8_t)(x & 0xFF);
    msg->payload[1]  = (uint8_t)((x >> 8) & 0xFF);
    msg->payload[2]  = (uint8_t)(y & 0xFF);
    msg->payload[3]  = (uint8_t)((y >> 8) & 0xFF);
    msg->payload[4]  = (uint8_t)(z & 0xFF);
    msg->payload[5]  = (uint8_t)((z >> 8) & 0xFF);
    msg->payload[6]  = (uint8_t)(r & 0xFF);
    msg->payload[7]  = (uint8_t)((r >> 8) & 0xFF);
    msg->payload[8]  = (uint8_t)(buttons & 0xFF);
    msg->payload[9]  = (uint8_t)((buttons >> 8) & 0xFF);
    msg->payload[10] = target;

    mavlink_finalize(msg, sysid, compid,
                     MAVLINK_MSG_MANUAL_CONTROL_LEN,
                     MAVLINK_MSG_ID_MANUAL_CONTROL,
                     MAVLINK_MSG_MANUAL_CONTROL_CRC,
                     seq);
}

#endif /* MAVLINK_MINIMAL_H */
