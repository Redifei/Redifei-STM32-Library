/*
 mpu6050.h - mpu6050 library
 This file is part of Redifei STM32 Library.

 Redifei STM32 Library is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Redifei STM32 Library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Redifei STM32 Library.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef enum {
  RED_MPU6050_1,
  RED_MPU6050_2,
  RED_MPU6050_MAX,
} red_mpu6050Devices;

typedef struct {
  // mpu6050I2c
  uint8_t i2cDeviceNum;
  red_i2c_userSetting_t* i2cUserSetting;

  // MPU6050_DLPF_BW
  uint8_t lpf;
  // MPU6050_GYRO_FS
  uint8_t gyroScale;
  // MPU6050_ACCEL_FS
  uint8_t accScale;
} red_mpu6050_userSetting_t;

typedef struct {
  red_i2cDevice_t* i2cDevice;
  red_mpu6050_userSetting_t* userSetting;
  uint8_t deviceAddr;
  float gyroScaleFactor;
  float accScaleFactor;
  float tempScaleFactor;
} red_mpu6050_param_t;

typedef struct red_mpu6050Port {
  red_mpu6050_param_t* param;

  void (*accInit)(struct red_mpu6050Port* this);
  bool (*accDetect)(struct red_mpu6050Port* this);
  bool (*accRead)(struct red_mpu6050Port* this, int16_t* buf);
  float (*accScaleFactor)(struct red_mpu6050Port* this); // 1G

  void (*gyroInit)(struct red_mpu6050Port* this);
  bool (*gyroDetect)(struct red_mpu6050Port* this);
  bool (*gyroRead)(struct red_mpu6050Port* this, int16_t* buf);
  float (*gyroScaleFactor)(struct red_mpu6050Port* this); // deg/s

  bool (*tempRead)(struct red_mpu6050Port* this, int16_t* buf);
} red_mpu6050Port_t;

#define MPU6050_ADDRESS_AD0_LOW         0x68
#define MPU6050_ADDRESS_AD0_HIGH        0x69

#define MPU6050_RA_CONFIG               0x1A
#define MPU6050_RA_GYRO_CONFIG          0x1B
#define MPU6050_RA_ACCEL_CONFIG         0x1C

#define MPU6050_RA_ACCEL_XOUT_H         0x3B
#define MPU6050_RA_ACCEL_XOUT_L         0x3C
#define MPU6050_RA_ACCEL_YOUT_H         0x3D
#define MPU6050_RA_ACCEL_YOUT_L         0x3E
#define MPU6050_RA_ACCEL_ZOUT_H         0x3F
#define MPU6050_RA_ACCEL_ZOUT_L         0x40
#define MPU6050_RA_TEMP_OUT_H           0x41
#define MPU6050_RA_TEMP_OUT_L           0x42
#define MPU6050_RA_GYRO_XOUT_H          0x43
#define MPU6050_RA_GYRO_XOUT_L          0x44
#define MPU6050_RA_GYRO_YOUT_H          0x45
#define MPU6050_RA_GYRO_YOUT_L          0x46
#define MPU6050_RA_GYRO_ZOUT_H          0x47
#define MPU6050_RA_GYRO_ZOUT_L          0x48

#define MPU6050_RA_PWR_MGMT_1           0x6B

#define MPU6050_RA_WHO_AM_I             0x75

/* CONFIG */
#define MPU6050_DLPF_BW_256             0x00
#define MPU6050_DLPF_BW_188             0x01
#define MPU6050_DLPF_BW_98              0x02
#define MPU6050_DLPF_BW_42              0x03
#define MPU6050_DLPF_BW_20              0x04
#define MPU6050_DLPF_BW_10              0x05
#define MPU6050_DLPF_BW_5               0x06

/* GYRO_CONFIG */
#define MPU6050_GYRO_FS_250             0x00
#define MPU6050_GYRO_FS_500             0x08
#define MPU6050_GYRO_FS_1000            0x10
#define MPU6050_GYRO_FS_2000            0x18

/* ACCEL_CONFIG */
#define MPU6050_ACCEL_FS_2              0x00
#define MPU6050_ACCEL_FS_4              0x08
#define MPU6050_ACCEL_FS_8              0x10
#define MPU6050_ACCEL_FS_16             0x18

/* PWR_MGMT_1 */
#define MPU6050_PWR1_DEVICE_RESET       0x80
#define MPU6050_PWR1_SLEEP              0x40
#define MPU6050_PWR1_CYCLE              0x20
#define MPU6050_PWR1_TEMP_DIS           0x08

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

/* WHO_AM_I */
#define MPU6050_WHO_AM_I_DEFAULT        0x68

#define IS_CONFIGED_MPU6050_PORT(THIS_SETTING) (THIS_SETTING != NULL)
#define IS_VAILD_MPU6050_PORT_NUM(MPU6050_NUM) (MPU6050_NUM < RED_MPU6050_MAX)


red_mpu6050Port_t* redMpu6050Init(uint8_t mpu6050PortNum, red_mpu6050_userSetting_t* userSetting);
