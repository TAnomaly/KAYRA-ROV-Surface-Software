/*
 * mpu6050.h — MPU-6050 (and compatible clones) I2C IMU driver
 *
 * Reads 3-axis accelerometer + 3-axis gyroscope.
 * Computes roll, pitch, yaw using complementary filter.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/* Default I2C address (AD0 low = 0x68, AD0 high = 0x69) */
#define MPU6050_ADDR    0x68

typedef struct {
    int      fd;            /* I2C file descriptor           */
    uint8_t  addr;          /* I2C slave address             */

    /* Raw sensor data */
    int16_t  accel_raw[3];  /* X, Y, Z  (accel)              */
    int16_t  gyro_raw[3];   /* X, Y, Z  (gyro)               */
    int16_t  temp_raw;      /* temperature                   */

    /* Calibration offsets (averaged at init) */
    float    gyro_offset[3];

    /* Filtered orientation (degrees) */
    float    roll;          /* degrees, + = right wing down  */
    float    pitch;         /* degrees, + = nose up          */
    float    yaw;           /* degrees, integrated from gyro */

    /* Temperature */
    float    temp_c;        /* Celsius                       */

    /* Timing */
    uint64_t last_update_us;
} mpu6050_t;

/* Initialise: open I2C bus, wake up sensor, calibrate gyro.
 * Returns 0 on success, -1 on failure.                      */
int mpu6050_init(mpu6050_t *imu, int bus, uint8_t addr);

/* Read raw data and update filtered roll/pitch/yaw.
 * Call this every main-loop iteration (~50 Hz or faster).   */
int mpu6050_update(mpu6050_t *imu);

/* Close the I2C file descriptor. */
void mpu6050_close(mpu6050_t *imu);

#endif /* MPU6050_H */
