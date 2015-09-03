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
#include <stdbool.h>
#include <stm32f10x_conf.h>
#include "systickTimer.h"
#include "i2c.h"
#include "mpu6050.h"

static i2cPort_t* mpu6050I2C;
static mpu6050_t myMPU6050;

static void reset() {
  uint8_t buf;
  mpu6050I2C->write1Byte(myMPU6050.deviceAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
  while (1) {
    delayMicroseconds(5);
    mpu6050I2C->read1Byte(myMPU6050.deviceAddr, MPU6050_RA_PWR_MGMT_1, &buf);
    if (!(buf & MPU6050_PWR1_DEVICE_RESET)) // until DEVICE_RESET bit for PWR_MGMT_1 to reset
      break;
  }
}
static uint8_t setClockSource(uint8_t clockSource) {
  return mpu6050I2C->write1Byte(myMPU6050.deviceAddr, MPU6050_RA_PWR_MGMT_1, clockSource);
}
static uint8_t setDLPFMode(uint8_t DLPFMode) {
  return mpu6050I2C->write1Byte(myMPU6050.deviceAddr, MPU6050_RA_CONFIG, DLPFMode);
}
static uint8_t setFullScaleGyroRange(uint8_t gyroRange) {
  return mpu6050I2C->write1Byte(myMPU6050.deviceAddr, MPU6050_RA_GYRO_CONFIG, gyroRange);
}
static uint8_t setFullScaleAccelRange(uint8_t accelRange) {
  return mpu6050I2C->write1Byte(myMPU6050.deviceAddr, MPU6050_RA_ACCEL_CONFIG, accelRange);
}

static void mpu6050_gyro_init() {
  reset();
  setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  setDLPFMode(MPU6050_DLPF_BW_20);
  setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
}
static bool mpu6050_gyro_detect() {
  uint8_t buf, i;
  bool detected = false;
  for (i = 0; i < 2; i++) {
    uint8_t addr = (i == 0) ? (MPU6050_ADDRESS_AD0_LOW) : (MPU6050_ADDRESS_AD0_HIGH);
    uint8_t err = mpu6050I2C->read1Byte(addr, MPU6050_RA_WHO_AM_I, &buf);
    if (buf == MPU6050_WHO_AM_I_DEFAULT && !err) { // Success Communicate and device is detected
      myMPU6050.deviceAddr = addr;
      detected = true;
    }
  }
  return detected;
}
static uint8_t mpu6050_gyro_read(int16_t* gyroRawData) {
  uint8_t buf[6], err;
  err = mpu6050I2C->readBytes(myMPU6050.deviceAddr, MPU6050_RA_GYRO_XOUT_H, 6, buf);
  gyroRawData[0] = (buf[0] << 8 | buf[1]);
  gyroRawData[1] = (buf[2] << 8 | buf[3]);
  gyroRawData[2] = (buf[4] << 8 | buf[5]);
  return err;
}

static void mpu6050_acc_init() {
  setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
}
static bool mpu6050_acc_detect() {
  return mpu6050_gyro_detect();
}
static uint8_t mpu6050_acc_read(int16_t* accRawData) {
  uint8_t buf[6], err;
  err = mpu6050I2C->readBytes(myMPU6050.deviceAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buf);
  accRawData[0] = (buf[0] << 8 | buf[1]);
  accRawData[1] = (buf[2] << 8 | buf[3]);
  accRawData[2] = (buf[4] << 8 | buf[5]);
  return err;
}

static uint8_t mpu6050_temp_read(int16_t* tempRawData) {
  uint8_t buf[6], err;
  err = mpu6050I2C->readBytes(myMPU6050.deviceAddr, MPU6050_RA_TEMP_OUT_H, 2, buf);
  tempRawData[0] = (buf[0] << 8 | buf[1]);
  return err;
}

mpu6050_t* mpu6050Config() {
  mpu6050I2C = openI2C1(I2C_INTERRPUT_MODE);
  myMPU6050.gyroInit = mpu6050_gyro_init;
  myMPU6050.gyroDetect = mpu6050_gyro_detect;
  myMPU6050.gyroRead = mpu6050_gyro_read;

  myMPU6050.accInit = mpu6050_acc_init;
  myMPU6050.accDetect = mpu6050_acc_detect;
  myMPU6050.accRead = mpu6050_acc_read;

  myMPU6050.tempRead = mpu6050_temp_read;
  return &myMPU6050;
}
