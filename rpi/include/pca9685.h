/*
 * pca9685.h — PCA9685 16-channel PWM driver over I2C
 *
 * Minimal driver for controlling ESCs via the PCA9685.
 * Uses Linux i2c-dev ioctl interface (/dev/i2c-N).
 */

#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>

typedef struct {
    int      fd;           /* i2c-dev file descriptor */
    uint8_t  addr;         /* I2C slave address       */
    float    pulse_scale;  /* ticks per microsecond    */
} pca9685_t;

/* Open I2C bus and initialise PCA9685 at given frequency.
 * Returns 0 on success, -1 on error. */
int  pca9685_init(pca9685_t *dev, int bus, uint8_t addr, int freq_hz);

/* Set a channel's pulse width in microseconds (e.g. 1500 for neutral). */
void pca9685_set_pulse_us(pca9685_t *dev, int channel, int pulse_us);

/* Set all 16 channels to the same pulse width. */
void pca9685_set_all_us(pca9685_t *dev, int pulse_us);

/* Put PCA9685 to sleep (outputs off). */
void pca9685_sleep(pca9685_t *dev);

/* Close I2C file descriptor. */
void pca9685_close(pca9685_t *dev);

#endif /* PCA9685_H */
