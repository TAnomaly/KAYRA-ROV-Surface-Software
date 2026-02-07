/*
 * transport.h — UDP and Serial transport abstraction
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    TRANSPORT_UDP,
    TRANSPORT_SERIAL
} transport_type_t;

typedef struct {
    transport_type_t type;
    int  fd;

    /* UDP */
    char target_ip[64];
    int  target_port;

    /* Serial */
    char device[256];
    int  baud_rate;
} transport_t;

/*  Open the transport.  Returns 0 on success.  */
int   transport_init(transport_t *t);

/*  Send len bytes.  Non-blocking.  Returns bytes sent or -1.  */
int   transport_send(transport_t *t, const uint8_t *buf, size_t len);

/*  Close the file descriptor.  */
void  transport_close(transport_t *t);

#endif /* TRANSPORT_H */
