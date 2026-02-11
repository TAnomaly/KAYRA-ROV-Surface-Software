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
    int  listen_port;      /* local bind port (0 = don't bind) */

    /* Serial */
    char device[256];
    int  baud_rate;
} transport_t;

/*  Open the transport.  Returns 0 on success.
 *  For UDP, if listen_port > 0 the socket is bound to that local port
 *  so incoming telemetry can be received via transport_recv().          */
int   transport_init(transport_t *t);

/*  Send len bytes.  Non-blocking.  Returns bytes sent or -1.  */
int   transport_send(transport_t *t, const uint8_t *buf, size_t len);

/*  Non-blocking receive.  Returns bytes read, 0 if nothing, -1 on err. */
int   transport_recv(transport_t *t, uint8_t *buf, size_t maxlen);

/*  Close the file descriptor.  */
void  transport_close(transport_t *t);

#endif /* TRANSPORT_H */
