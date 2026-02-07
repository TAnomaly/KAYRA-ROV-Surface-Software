/*
 * mavlink_packer.h — Map joystick state to MAVLink messages
 */

#ifndef MAVLINK_PACKER_H
#define MAVLINK_PACKER_H

#include <stddef.h>
#include <stdint.h>
#include "joystick.h"

/* Processed control values ready for MAVLink packing */
typedef struct {
    int16_t  x;         /* surge:  forward (+) / backward (-) */
    int16_t  y;         /* sway:   right (+)   / left (-)     */
    int16_t  z;         /* heave:  up (+)      / down (-)     */
    int16_t  r;         /* yaw:    CW (+)      / CCW (-)      */
    uint16_t buttons;
} manual_control_t;

void     mavlink_packer_init(void);

/*  Map normalised joystick axes → MANUAL_CONTROL fields.
 *  Applies axis assignment, inversion, and clamping.          */
void     mavlink_packer_map(const joystick_state_t *js,
                            manual_control_t *ctrl);

/*  Pack a MANUAL_CONTROL frame into buf.  Returns frame length,
 *  or 0 if buf_len is too small.                                */
uint16_t mavlink_packer_pack_manual_control(const manual_control_t *ctrl,
                                            uint8_t *buf, size_t buf_len);

/*  Pack a GCS HEARTBEAT frame into buf.  Returns frame length.  */
uint16_t mavlink_packer_pack_heartbeat(uint8_t *buf, size_t buf_len);

#endif /* MAVLINK_PACKER_H */
