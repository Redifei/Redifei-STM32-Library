/*
 * Redifei: MPU6050 Library
 * Based on i2cdevlib Project
 *
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 *
 *
 * -Functions
 * mpu6050Config
 * ->accInit
 * ->accDetect
 * ->accRead
 * ->gyroInit
 * ->gyroDetect
 * ->gyroRead
 * ->tempRead
 */
#pragma once

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

typedef struct {
  uint8_t (*write1Byte)(uint8_t addr, uint8_t reg, uint8_t data);
  uint8_t (*readBytes)(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
  uint8_t (*read1Byte)(uint8_t addr, uint8_t reg, uint8_t* buf);
} i2cMpu6050_t;

typedef struct {
  uint8_t deviceAddr;

  i2cMpu6050_t i2c;

  void (*accInit)();
  bool (*accDetect)();
  uint8_t (*accRead)(int16_t* buf);

  void (*gyroInit)();
  bool (*gyroDetect)();
  uint8_t (*gyroRead)(int16_t* buf);

  uint8_t (*tempRead)(int16_t* buf);
} mpu6050_t;

mpu6050_t* mpu6050Config();
