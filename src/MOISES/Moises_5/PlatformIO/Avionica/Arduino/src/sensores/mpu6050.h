/*
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   Interface do Driver para o sensor MPU6050
  ******************************************************************************
*/

#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include <macros_config.h>
#include <utils/utils.h>

constexpr u8 MPU_STATUS_VALID           = 0x01;
constexpr u8 MPU_STATUS_GYRO_CALIBRATED = 0x02;

typedef struct {
    i16  accelRaw[3];
    i16  gyroRaw[3];
    i16  gyroBiasRaw[3];
    u8   status;
    bool online;
    bool gyroCalibrated;
} MPU6050_t;

bool mpu6050Init(MPU6050_t *imu);
bool mpu6050CalibrateGyro(MPU6050_t *imu);
void mpu6050Acquire(MPU6050_t *imu, u32 nowMs);

#endif /* MPU6050_DRIVER_H */