/*
 * mpu6050.c — MPU-6050 I2C driver with complementary filter
 *
 * Compatible with genuine MPU-6050 and common clones (WHO_AM_I = 0x98 etc.)
 *
 * Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/
 *               MPU-6000-Register-Map1.pdf
 */

#include "mpu6050.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* ── MPU-6050 registers ── */
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B   /* 14 bytes: accel(6) + temp(2) + gyro(6) */
#define REG_PWR_MGMT_1    0x6B
#define REG_PWR_MGMT_2    0x6C
#define REG_WHO_AM_I      0x75

/* ── Sensitivity scales ── */
/* Accel: ±2g  → 16384 LSB/g  */
#define ACCEL_SCALE       16384.0f
/* Gyro:  ±250°/s → 131 LSB/(°/s) */
#define GYRO_SCALE        131.0f

/* Complementary filter coefficient (0.98 = trust gyro 98%, accel 2%) */
#define ALPHA             0.98f

/* Number of samples for gyro calibration */
#define CALIB_SAMPLES     200

/* ── Helpers ── */

static int i2c_write(int fd, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    if (write(fd, buf, 2) != 2) return -1;
    return 0;
}

static int i2c_read_bytes(int fd, uint8_t reg, uint8_t *data, int len)
{
    if (write(fd, &reg, 1) != 1) return -1;
    if (read(fd, data, (size_t)len) != len) return -1;
    return 0;
}

static int16_t to_i16(uint8_t h, uint8_t l)
{
    return (int16_t)((uint16_t)h << 8 | l);
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* ── Public API ── */

int mpu6050_init(mpu6050_t *imu, int bus, uint8_t addr)
{
    memset(imu, 0, sizeof(*imu));
    imu->fd = -1;
    imu->addr = addr;

    /* Open I2C bus */
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
    imu->fd = open(path, O_RDWR);
    if (imu->fd < 0) {
        perror("[mpu6050] open i2c");
        return -1;
    }

    if (ioctl(imu->fd, I2C_SLAVE, addr) < 0) {
        perror("[mpu6050] ioctl I2C_SLAVE");
        close(imu->fd); imu->fd = -1;
        return -1;
    }

    /* Check WHO_AM_I (accept any non-zero value — clones differ) */
    uint8_t who = 0;
    i2c_read_bytes(imu->fd, REG_WHO_AM_I, &who, 1);
    printf("[mpu6050] WHO_AM_I = 0x%02X (addr=0x%02X bus=%d)\n",
           who, addr, bus);

    if (who == 0x00 || who == 0xFF) {
        fprintf(stderr, "[mpu6050] Sensor not responding\n");
        close(imu->fd); imu->fd = -1;
        return -1;
    }

    /* Wake up (clear SLEEP bit) + use X-axis gyro as clock */
    i2c_write(imu->fd, REG_PWR_MGMT_1, 0x01);
    usleep(100000);  /* 100ms settle */

    /* Sample rate = 1kHz / (1 + SMPLRT_DIV) = 50 Hz */
    i2c_write(imu->fd, REG_SMPLRT_DIV, 19);

    /* DLPF config: ~44Hz bandwidth (good for vibration filtering) */
    i2c_write(imu->fd, REG_CONFIG, 0x03);

    /* Gyro: ±250°/s */
    i2c_write(imu->fd, REG_GYRO_CONFIG, 0x00);

    /* Accel: ±2g */
    i2c_write(imu->fd, REG_ACCEL_CONFIG, 0x00);

    /* ── Gyro calibration: average N samples at rest ── */
    printf("[mpu6050] Calibrating gyro (%d samples)...\n", CALIB_SAMPLES);

    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint8_t raw[14];

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        if (i2c_read_bytes(imu->fd, REG_ACCEL_XOUT_H, raw, 14) < 0)
            continue;
        gx_sum += (float)to_i16(raw[8],  raw[9]);
        gy_sum += (float)to_i16(raw[10], raw[11]);
        gz_sum += (float)to_i16(raw[12], raw[13]);
        usleep(5000);  /* 5ms between samples */
    }

    imu->gyro_offset[0] = gx_sum / CALIB_SAMPLES;
    imu->gyro_offset[1] = gy_sum / CALIB_SAMPLES;
    imu->gyro_offset[2] = gz_sum / CALIB_SAMPLES;

    printf("[mpu6050] Gyro offsets: %.1f  %.1f  %.1f\n",
           imu->gyro_offset[0], imu->gyro_offset[1], imu->gyro_offset[2]);

    /* Initialize with accelerometer-only angles */
    if (i2c_read_bytes(imu->fd, REG_ACCEL_XOUT_H, raw, 14) == 0) {
        float ax = (float)to_i16(raw[0], raw[1]) / ACCEL_SCALE;
        float ay = (float)to_i16(raw[2], raw[3]) / ACCEL_SCALE;
        float az = (float)to_i16(raw[4], raw[5]) / ACCEL_SCALE;

        imu->roll  = atan2f(ay, az) * (180.0f / (float)M_PI);
        imu->pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);
        imu->yaw   = 0.0f;
    }

    imu->last_update_us = now_us();

    printf("[mpu6050] Init OK — roll=%.1f° pitch=%.1f°\n",
           imu->roll, imu->pitch);
    return 0;
}

int mpu6050_update(mpu6050_t *imu)
{
    if (imu->fd < 0) return -1;

    /* Read 14 bytes: accel(6) + temp(2) + gyro(6) */
    uint8_t raw[14];
    if (i2c_read_bytes(imu->fd, REG_ACCEL_XOUT_H, raw, 14) < 0)
        return -1;

    /* Parse raw values */
    imu->accel_raw[0] = to_i16(raw[0],  raw[1]);
    imu->accel_raw[1] = to_i16(raw[2],  raw[3]);
    imu->accel_raw[2] = to_i16(raw[4],  raw[5]);
    imu->temp_raw      = to_i16(raw[6],  raw[7]);
    imu->gyro_raw[0]  = to_i16(raw[8],  raw[9]);
    imu->gyro_raw[1]  = to_i16(raw[10], raw[11]);
    imu->gyro_raw[2]  = to_i16(raw[12], raw[13]);

    /* Temperature: T(°C) = raw / 340 + 36.53 */
    imu->temp_c = (float)imu->temp_raw / 340.0f + 36.53f;

    /* Convert to physical units */
    float ax = (float)imu->accel_raw[0] / ACCEL_SCALE;
    float ay = (float)imu->accel_raw[1] / ACCEL_SCALE;
    float az = (float)imu->accel_raw[2] / ACCEL_SCALE;

    float gx = ((float)imu->gyro_raw[0] - imu->gyro_offset[0]) / GYRO_SCALE;
    float gy = ((float)imu->gyro_raw[1] - imu->gyro_offset[1]) / GYRO_SCALE;
    float gz = ((float)imu->gyro_raw[2] - imu->gyro_offset[2]) / GYRO_SCALE;

    /* Delta time in seconds */
    uint64_t now = now_us();
    float dt = (float)(now - imu->last_update_us) / 1000000.0f;
    imu->last_update_us = now;

    /* Clamp dt to prevent spikes */
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;

    /* Accelerometer angles */
    float accel_roll  = atan2f(ay, az) * (180.0f / (float)M_PI);
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);

    /* Complementary filter: fuse gyro (fast) + accel (drift-free) */
    imu->roll  = ALPHA * (imu->roll  + gx * dt) + (1.0f - ALPHA) * accel_roll;
    imu->pitch = ALPHA * (imu->pitch + gy * dt) + (1.0f - ALPHA) * accel_pitch;

    /* Yaw: gyro integration only (no magnetometer → will drift slowly) */
    imu->yaw += gz * dt;
    /* Wrap to 0..360 */
    if (imu->yaw < 0.0f)   imu->yaw += 360.0f;
    if (imu->yaw >= 360.0f) imu->yaw -= 360.0f;

    return 0;
}

void mpu6050_close(mpu6050_t *imu)
{
    if (imu->fd >= 0) {
        /* Put sensor back to sleep */
        i2c_write(imu->fd, REG_PWR_MGMT_1, 0x40);
        close(imu->fd);
        imu->fd = -1;
    }
}
