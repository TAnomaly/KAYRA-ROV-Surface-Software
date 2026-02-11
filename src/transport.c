/*
 * transport.c — UDP and Serial (UART-over-USB) transport
 *
 * Both transports use non-blocking I/O so the main loop never stalls
 * waiting for the OS to accept bytes.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

/* UDP */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* Serial */
#include <termios.h>

#include "transport.h"

/* ================================================================== */
/*  UDP                                                               */
/* ================================================================== */

static int udp_init(transport_t *t)
{
    t->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (t->fd < 0) {
        perror("[transport] socket()");
        return -1;
    }

    /* Non-blocking */
    int flags = fcntl(t->fd, F_GETFL, 0);
    if (flags < 0 || fcntl(t->fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("[transport] fcntl non-block");
        close(t->fd);
        t->fd = -1;
        return -1;
    }

    /* Bind to local port so we can receive telemetry from RPi */
    if (t->listen_port > 0) {
        struct sockaddr_in bind_addr;
        memset(&bind_addr, 0, sizeof(bind_addr));
        bind_addr.sin_family      = AF_INET;
        bind_addr.sin_port        = htons((uint16_t)t->listen_port);
        bind_addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(t->fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
            perror("[transport] bind()");
            close(t->fd);
            t->fd = -1;
            return -1;
        }
        printf("[transport] UDP bound to 0.0.0.0:%d  (recv enabled)\n",
               t->listen_port);
    }

    printf("[transport] UDP → %s:%d  (fd %d)\n",
           t->target_ip, t->target_port, t->fd);
    return 0;
}

static int udp_send(transport_t *t, const uint8_t *buf, size_t len)
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons((uint16_t)t->target_port);

    if (inet_pton(AF_INET, t->target_ip, &addr.sin_addr) != 1) {
        fprintf(stderr, "[transport] bad IP: %s\n", t->target_ip);
        return -1;
    }

    ssize_t n = sendto(t->fd, buf, len, 0,
                       (struct sockaddr *)&addr, sizeof(addr));
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        perror("[transport] sendto()");
        return -1;
    }
    return (int)n;
}

/* ================================================================== */
/*  Serial  (UART over USB — /dev/ttyUSB0 or /dev/ttyACM0)           */
/* ================================================================== */

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:
        fprintf(stderr, "[transport] unknown baud %d, using 115200\n", baud);
        return B115200;
    }
}

static int serial_init(transport_t *t)
{
    t->fd = open(t->device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (t->fd < 0) {
        perror("[transport] open(serial)");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(t->fd, &tty) != 0) {
        perror("[transport] tcgetattr");
        close(t->fd);
        t->fd = -1;
        return -1;
    }

    speed_t spd = baud_to_speed(t->baud_rate);
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    /* 8N1 — no parity, one stop bit, no HW flow control */
    tty.c_cflag  = (tty.c_cflag & (unsigned)~CSIZE) | CS8;
    tty.c_cflag &= (unsigned)~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CLOCAL | CREAD;

    /* Raw input — no echo, no canonical processing */
    tty.c_iflag &= (unsigned)~(IXON | IXOFF | IXANY |
                                IGNBRK | BRKINT | PARMRK |
                                ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= (unsigned)~OPOST;
    tty.c_lflag &= (unsigned)~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN]  = 0;   /* non-blocking read */
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(t->fd, TCSANOW, &tty) != 0) {
        perror("[transport] tcsetattr");
        close(t->fd);
        t->fd = -1;
        return -1;
    }

    printf("[transport] Serial → %s @ %d baud  (fd %d)\n",
           t->device, t->baud_rate, t->fd);
    return 0;
}

static int serial_send(transport_t *t, const uint8_t *buf, size_t len)
{
    ssize_t n = write(t->fd, buf, len);
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        perror("[transport] write(serial)");
        return -1;
    }
    return (int)n;
}

/* ================================================================== */
/*  Non-blocking receive (UDP or Serial)                              */
/* ================================================================== */

static int udp_recv(transport_t *t, uint8_t *buf, size_t maxlen)
{
    ssize_t n = recvfrom(t->fd, buf, maxlen, MSG_DONTWAIT, NULL, NULL);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return 0;           /* nothing available */
        return -1;
    }
    return (int)n;
}

static int serial_recv(transport_t *t, uint8_t *buf, size_t maxlen)
{
    ssize_t n = read(t->fd, buf, maxlen);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return 0;
        return -1;
    }
    return (int)n;
}

/* ================================================================== */
/*  Public interface                                                  */
/* ================================================================== */

int transport_init(transport_t *t)
{
    switch (t->type) {
    case TRANSPORT_UDP:    return udp_init(t);
    case TRANSPORT_SERIAL: return serial_init(t);
    }
    return -1;
}

int transport_send(transport_t *t, const uint8_t *buf, size_t len)
{
    switch (t->type) {
    case TRANSPORT_UDP:    return udp_send(t, buf, len);
    case TRANSPORT_SERIAL: return serial_send(t, buf, len);
    }
    return -1;
}

int transport_recv(transport_t *t, uint8_t *buf, size_t maxlen)
{
    switch (t->type) {
    case TRANSPORT_UDP:    return udp_recv(t, buf, maxlen);
    case TRANSPORT_SERIAL: return serial_recv(t, buf, maxlen);
    }
    return -1;
}

void transport_close(transport_t *t)
{
    if (t->fd >= 0) {
        close(t->fd);
        t->fd = -1;
    }
    printf("[transport] Closed.\n");
}
