/*
 * Redifei: MPU6050 Library
 * Based on i2cdevlib Project
 *
 ******************************************************************************
 * This file is part of Redifei Library.
 *
 * Redifei Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Redifei Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Redifei Library.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/*
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

static mpu6050_t myMPU6050;

static void mpu6050_gyro_init();
static bool mpu6050_gyro_detect();
static uint8_t mpu6050_gyro_read(int16_t* gyroRawData);
static void mpu6050_acc_init();
static bool mpu6050_acc_detect();
static uint8_t mpu6050_acc_read(int16_t* accRawData);
static uint8_t mpu6050_temp_read(int16_t* tempRawData);

/****************************************
 * Internal Functions
 ****************************************/
static void _reset(mpu6050_t* instant) {
  uint8_t buf;
  instant->i2c.write1Byte(instant->deviceAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
  while (1) {
    delayMicroseconds(5);
    instant->i2c.read1Byte(instant->deviceAddr, MPU6050_RA_PWR_MGMT_1, &buf);
    if (!(buf & MPU6050_PWR1_DEVICE_RESET)) // until DEVICE_RESET bit for PWR_MGMT_1 to reset
      break;
  }
}
static uint8_t _setClockSource(mpu6050_t* instant, uint8_t clockSource) {
  return instant->i2c.write1Byte(instant->deviceAddr, MPU6050_RA_PWR_MGMT_1, clockSource);
}
static uint8_t _setDLPFMode(mpu6050_t* instant, uint8_t DLPFMode) {
  return instant->i2c.write1Byte(instant->deviceAddr, MPU6050_RA_CONFIG, DLPFMode);
}
static uint8_t _setFullScaleGyroRange(mpu6050_t* instant, uint8_t gyroRange) {
  return instant->i2c.write1Byte(instant->deviceAddr, MPU6050_RA_GYRO_CONFIG, gyroRange);
}
static uint8_t _setFullScaleAccelRange(mpu6050_t* instant, uint8_t accelRange) {
  return instant->i2c.write1Byte(instant->deviceAddr, MPU6050_RA_ACCEL_CONFIG, accelRange);
}
static void _mpu6050_gyro_init(mpu6050_t* instant) {
  _reset(instant);
  _setClockSource(instant, MPU6050_CLOCK_PLL_XGYRO);
  _setDLPFMode(instant, MPU6050_DLPF_BW_20);
  _setFullScaleGyroRange(instant, MPU6050_GYRO_FS_2000);
}
static bool _mpu6050_gyro_detect(mpu6050_t* instant) {
  uint8_t buf, i;
  bool detected = false;
  for (i = 0; i < 2; i++) {
    uint8_t addr = (i == 0) ? (MPU6050_ADDRESS_AD0_LOW) : (MPU6050_ADDRESS_AD0_HIGH);
    uint8_t err = instant->i2c.read1Byte(addr, MPU6050_RA_WHO_AM_I, &buf);
    if (buf == MPU6050_WHO_AM_I_DEFAULT && !err) { // Success Communicate and device is detected
      instant->deviceAddr = addr;
      detected = true;
    }
  }
  return detected;
}
static uint8_t _mpu6050_gyro_read(mpu6050_t* instant, int16_t* gyroRawData) {
  uint8_t buf[6], err;
  err = instant->i2c.readBytes(instant->deviceAddr, MPU6050_RA_GYRO_XOUT_H, 6, buf);
  gyroRawData[0] = (buf[0] << 8 | buf[1]);
  gyroRawData[1] = (buf[2] << 8 | buf[3]);
  gyroRawData[2] = (buf[4] << 8 | buf[5]);
  return err;
}

static void _mpu6050_acc_init(mpu6050_t* instant) {
  _setFullScaleAccelRange(instant, MPU6050_ACCEL_FS_16);
}
static bool _mpu6050_acc_detect(mpu6050_t* instant) {
  return _mpu6050_gyro_detect(instant);
}
static uint8_t _mpu6050_acc_read(mpu6050_t* instant, int16_t* accRawData) {
  uint8_t buf[6], err;
  err = instant->i2c.readBytes(instant->deviceAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buf);
  accRawData[0] = (buf[0] << 8 | buf[1]);
  accRawData[1] = (buf[2] << 8 | buf[3]);
  accRawData[2] = (buf[4] << 8 | buf[5]);
  return err;
}

static uint8_t _mpu6050_temp_read(mpu6050_t* instant, int16_t* tempRawData) {
  uint8_t buf[6], err;
  err = instant->i2c.readBytes(instant->deviceAddr, MPU6050_RA_TEMP_OUT_H, 2, buf);
  tempRawData[0] = (buf[0] << 8 | buf[1]);
  return err;
}

/****************************************
 * External Functions
 ****************************************/
/**
 * mpu6050Config
 * @note I2C1 | SCL : PB6, SDA : PB7, Speed : 400k
 * @return mpu6050_t*
 */
mpu6050_t* mpu6050Config() {
  i2cPort_t* mpu6050I2C = openI2C1(I2C_INTERRPUT_MODE);
  myMPU6050.i2c.read1Byte = mpu6050I2C->read1Byte;
  myMPU6050.i2c.readBytes = mpu6050I2C->readBytes;
  myMPU6050.i2c.write1Byte = mpu6050I2C->write1Byte;

  myMPU6050.gyroInit = mpu6050_gyro_init;
  myMPU6050.gyroDetect = mpu6050_gyro_detect;
  myMPU6050.gyroRead = mpu6050_gyro_read;

  myMPU6050.accInit = mpu6050_acc_init;
  myMPU6050.accDetect = mpu6050_acc_detect;
  myMPU6050.accRead = mpu6050_acc_read;

  myMPU6050.tempRead = mpu6050_temp_read;
  return &myMPU6050;
}

static void mpu6050_gyro_init() {
  _mpu6050_gyro_init(&myMPU6050);
}
static bool mpu6050_gyro_detect() {
  return _mpu6050_gyro_detect(&myMPU6050);
}
static uint8_t mpu6050_gyro_read(int16_t* gyroRawData) {
  return _mpu6050_gyro_read(&myMPU6050, gyroRawData);
}
static void mpu6050_acc_init() {
  return _mpu6050_acc_init(&myMPU6050);
}
static bool mpu6050_acc_detect() {
  return _mpu6050_acc_detect(&myMPU6050);
}
static uint8_t mpu6050_acc_read(int16_t* accRawData) {
  return _mpu6050_acc_read(&myMPU6050, accRawData);
}
static uint8_t mpu6050_temp_read(int16_t* tempRawData) {
  return _mpu6050_temp_read(&myMPU6050, tempRawData);
}
