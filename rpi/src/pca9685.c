/*
 * pca9685.c — PCA9685 PWM driver via Linux i2c-dev
 *
 * Register map: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
 */

#include "pca9685.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* ── PCA9685 Registers ── */
#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_PRESCALE    0xFE
#define REG_LED0_ON_L   0x06
#define REG_ALL_ON_L    0xFA

/* Mode1 bits */
#define MODE1_RESTART   0x80
#define MODE1_SLEEP     0x10
#define MODE1_AI        0x20    /* auto-increment */

/* Mode2 bits */
#define MODE2_OUTDRV    0x04    /* totem-pole output (required for ESCs) */

/* ── I2C helpers ── */

static int i2c_write_reg(int fd, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    if (write(fd, buf, 2) != 2) {
        perror("[pca9685] i2c write");
        return -1;
    }
    return 0;
}

static int i2c_read_reg(int fd, uint8_t reg)
{
    if (write(fd, &reg, 1) != 1) return -1;
    uint8_t val;
    if (read(fd, &val, 1) != 1) return -1;
    return val;
}

/* ── Public API ── */

int pca9685_init(pca9685_t *dev, int bus, uint8_t addr, int freq_hz)
{
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", bus);

    dev->fd = open(path, O_RDWR);
    if (dev->fd < 0) {
        perror("[pca9685] open i2c bus");
        return -1;
    }

    dev->addr = addr;
    if (ioctl(dev->fd, I2C_SLAVE, addr) < 0) {
        perror("[pca9685] ioctl I2C_SLAVE");
        close(dev->fd);
        dev->fd = -1;
        return -1;
    }

    /* MODE2: totem-pole output — required for ESC signal levels */
    i2c_write_reg(dev->fd, REG_MODE2, MODE2_OUTDRV);

    /* Calculate prescaler: prescale = round(25MHz / (4096 * freq)) - 1 */
    float prescale_f = 25000000.0f / (4096.0f * (float)freq_hz) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);

    /* Must sleep to change prescaler */
    if (i2c_write_reg(dev->fd, REG_MODE1, MODE1_SLEEP) < 0 ||
        i2c_write_reg(dev->fd, REG_PRESCALE, prescale) < 0 ||
        i2c_write_reg(dev->fd, REG_MODE1, MODE1_AI) < 0) {
        perror("[pca9685] init write failed");
        close(dev->fd); dev->fd = -1; return -1;
    }
    usleep(500);
    i2c_write_reg(dev->fd, REG_MODE1, MODE1_AI | MODE1_RESTART);
    usleep(500);

    /* ticks per microsecond: 4096 ticks per (1/freq) seconds */
    float period_us = 1000000.0f / (float)freq_hz;
    dev->pulse_scale = 4096.0f / period_us;

    printf("[pca9685] Init OK  bus=%d addr=0x%02X freq=%dHz prescale=%d\n",
           bus, addr, freq_hz, prescale);
    return 0;
}

void pca9685_set_pulse_us(pca9685_t *dev, int channel, int pulse_us)
{
    if (dev->fd < 0 || channel < 0 || channel > 15) return;

    uint16_t on  = 0;
    uint16_t off = (uint16_t)((float)pulse_us * dev->pulse_scale);
    if (off > 4095) off = 4095;

    uint8_t reg = REG_LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {
        reg,
        (uint8_t)(on & 0xFF),
        (uint8_t)((on >> 8) & 0x0F),
        (uint8_t)(off & 0xFF),
        (uint8_t)((off >> 8) & 0x0F),
    };

    if (write(dev->fd, buf, 5) != 5)
        perror("[pca9685] set pulse");
}

void pca9685_set_all_us(pca9685_t *dev, int pulse_us)
{
    for (int i = 0; i < 16; i++)
        pca9685_set_pulse_us(dev, i, pulse_us);
}

void pca9685_sleep(pca9685_t *dev)
{
    if (dev->fd < 0) return;
    int mode1 = i2c_read_reg(dev->fd, REG_MODE1);
    if (mode1 >= 0)
        i2c_write_reg(dev->fd, REG_MODE1, (uint8_t)(mode1 | MODE1_SLEEP));
}

void pca9685_close(pca9685_t *dev)
{
    if (dev->fd >= 0) {
        pca9685_sleep(dev);
        close(dev->fd);
        dev->fd = -1;
    }
}
