/*
 mpu6050.c - mpu6050 library
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

#include <stdbool.h>
#include <stddef.h>
#include <stm32f10x_conf.h>
#include "systickTimer.h"
#include "i2c.h"
#include "mpu6050.h"

static red_mpu6050Port_t mpu6050Ports[RED_MPU6050_MAX];
static red_mpu6050_param_t mpu6050Settings[RED_MPU6050_MAX];

static bool mpu6050WriteBytes(struct red_mpu6050Port* this, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = this->param;
  red_i2cDevice_t* i2c = param->i2cDevice;

  // 버퍼를 1칸씩 뒤로 물리고, 앞에 reg넣기
  uint8_t i;
  for(i = 0; i < len; i++) {
    uint8_t idx = len - i;
    buf[idx] = buf[idx-1];
  }
  buf[0] = reg;
  bool err = i2c->writeBytes(i2c, addr, len+1, buf);
  return err;
}
static bool mpu6050Write1Byte(struct red_mpu6050Port* this, uint8_t addr, uint8_t reg, uint8_t buf) {
  uint8_t _buf[2] = { buf, 0 };
  return mpu6050WriteBytes(this, addr, reg, 1, _buf);
}
static bool mpu6050ReadBytes(struct red_mpu6050Port* this, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = this->param;
  red_i2cDevice_t* i2c = param->i2cDevice;

  bool err = i2c->write1Byte(i2c, addr, reg);
  err |= i2c->readBytes(i2c, addr, len, buf);
  return err;
}
static bool mpu6050Read1Byte(struct red_mpu6050Port* this, uint8_t addr, uint8_t reg, uint8_t* buf) {
  return mpu6050ReadBytes(this, addr, reg, 1, buf);
}

static void red_setReset(struct red_mpu6050Port* this) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = this->param;
  red_i2cDevice_t* i2c = param->i2cDevice;
  red_mpu6050_userSetting_t* userSetting = param->userSetting;

  uint8_t buf;
  uint8_t err = mpu6050Write1Byte(this, param->deviceAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
  uint32_t timeOut = 0xff;
  while (1) {
    delayMicroseconds(5);
    err = mpu6050Read1Byte(this, param->deviceAddr, MPU6050_RA_PWR_MGMT_1, &buf);
    if (!(buf & MPU6050_PWR1_DEVICE_RESET)) // until DEVICE_RESET bit for PWR_MGMT_1 to reset
      break;
    if(timeOut-- == 0)
      return;
  }
}
static bool red_setClockSource(struct red_mpu6050Port* this, uint8_t clockSource) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = this->param;
  return mpu6050Write1Byte(this, param->deviceAddr, MPU6050_RA_PWR_MGMT_1, clockSource);
}
static bool red_setDLPFMode(struct red_mpu6050Port* this, uint8_t DLPFMode) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = this->param;
  return mpu6050Write1Byte(this, param->deviceAddr, MPU6050_RA_CONFIG, DLPFMode);
}
static bool red_setFullScaleGyroRange(struct red_mpu6050Port* this, uint8_t gyroRange) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = param;

  switch(gyroRange) {
  case MPU6050_GYRO_FS_250:
    param->gyroScaleFactor = 131.0f;
    break;
  case MPU6050_GYRO_FS_500:
    param->gyroScaleFactor = 65.5f;
    break;
  case MPU6050_GYRO_FS_1000:
    param->gyroScaleFactor = 32.8f;
    break;
  case MPU6050_GYRO_FS_2000:
    param->gyroScaleFactor = 16.4f;
    break;
  }
  return mpu6050Write1Byte(this, param->deviceAddr, MPU6050_RA_GYRO_CONFIG, gyroRange);
}
static bool red_setFullScaleAccelRange(struct red_mpu6050Port* this, uint8_t accelRange) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = this->param;

  switch(accelRange) {
  case MPU6050_ACCEL_FS_2:
    param->accScaleFactor = 16384.0f;
    break;
  case MPU6050_ACCEL_FS_4:
    param->accScaleFactor = 8192.0f;
    break;
  case MPU6050_ACCEL_FS_8:
    param->accScaleFactor = 4096.0f;
    break;
  case MPU6050_ACCEL_FS_16:
    param->accScaleFactor = 2048.0f;
    break;
  }
  return mpu6050Write1Byte(this, param->deviceAddr, MPU6050_RA_ACCEL_CONFIG, accelRange);
}
static void red_mpu6050GyroInit(struct red_mpu6050Port* this) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = this->param;
  red_mpu6050_userSetting_t* userSetting = param->userSetting;

  red_setReset(this);
  red_setClockSource(this, MPU6050_CLOCK_PLL_XGYRO);
  red_setDLPFMode(this, userSetting->lpf);
  red_setFullScaleGyroRange(this, userSetting->gyroScale); // scale factor : 16.4
}
static bool red_mpu6050GyroDetect(struct red_mpu6050Port* this) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));

  red_mpu6050_param_t* param = this->param;
  red_i2cDevice_t* i2c = param->i2cDevice;

  uint8_t buf, i;
  bool detected = false;
  for (i = 0; i < 2; i++) {
    uint8_t addr = (i == 0) ? (MPU6050_ADDRESS_AD0_LOW) : (MPU6050_ADDRESS_AD0_HIGH);
    bool err = mpu6050Read1Byte(this, addr, MPU6050_RA_WHO_AM_I, &buf);
    if (buf == MPU6050_WHO_AM_I_DEFAULT && !err) { // Success Communicate and device is detected
      param->deviceAddr = addr;
      detected = true;
      break;
    }
  }
  return detected;
}
static bool red_mpu6050GyroRead(struct red_mpu6050Port* this, int16_t* gyroRawData) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));

  red_mpu6050_param_t* param = this->param;
  red_i2cDevice_t* i2c = param->i2cDevice;
  red_mpu6050_userSetting_t* userSetting = param->userSetting;

  uint8_t buf[6];
  bool err = mpu6050ReadBytes(this, param->deviceAddr, MPU6050_RA_GYRO_XOUT_H, 6, buf);
  if(err) {
    i2c->reset(i2c);
    return true;
  }
  gyroRawData[0] = (buf[0] << 8 | buf[1]);
  gyroRawData[1] = (buf[2] << 8 | buf[3]);
  gyroRawData[2] = (buf[4] << 8 | buf[5]);
  return err;
}
static float red_mpu6050GyroScalefactor(struct red_mpu6050Port* this) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = this->param;
  return param->gyroScaleFactor;
}

static void red_mpu6050AccInit(struct red_mpu6050Port* this) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = this->param;
  red_mpu6050_userSetting_t* userSetting = param->userSetting;

  red_setFullScaleAccelRange(this, userSetting->accScale); // scale factor : 2048
}
static bool red_mpu6050AccDetect(struct red_mpu6050Port* this) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  return red_mpu6050GyroDetect(this);
}
static bool red_mpu6050AccRead(struct red_mpu6050Port* this, int16_t* accRawData) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));

  red_mpu6050_param_t* param = this->param;
  red_i2cDevice_t* i2c = param->i2cDevice;

  uint8_t buf[6];
  bool err = mpu6050ReadBytes(this, param->deviceAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buf);
  if(err) {
    i2c->reset(i2c);
    return true;
  }
  accRawData[0] = (buf[0] << 8 | buf[1]);
  accRawData[1] = (buf[2] << 8 | buf[3]);
  accRawData[2] = (buf[4] << 8 | buf[5]);
  return err;
}
static float red_mpu6050AccScalefactor(struct red_mpu6050Port* this) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  red_mpu6050_param_t* param = this->param;
  return param->accScaleFactor;
}

static bool red_mpu6050TempRead(struct red_mpu6050Port* this, int16_t* tempRawData) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));

  red_mpu6050_param_t* param = this->param;
  red_i2cDevice_t* i2c = param->i2cDevice;
  red_mpu6050_userSetting_t* userSetting = param->userSetting;

  uint8_t buf[2];
  bool err = mpu6050ReadBytes(this, param->deviceAddr, MPU6050_RA_TEMP_OUT_H, 2, buf);
  if(err) {
    i2c->reset(i2c);
    return true;
  }
  tempRawData[0] = (buf[0] << 8 | buf[1]);
  return err;
}

static void red_mpu6050Config(struct red_mpu6050Port* this) {
  assert_param(IS_CONFIGED_MPU6050_PORT(this->param));
  uint32_t timeOut = 0xffff;
  while(!red_mpu6050GyroDetect(this)) {
    if(timeOut-- == 0)
      return;
  }
  red_mpu6050GyroInit(this);
//  timeOut = 0xffff;
//  while(!red_mpu6050AccDetect(this)) {
//    if(timeOut-- == 0)
//      return;
//  }
  red_mpu6050AccInit(this);
}

/****************************************
 * External Functions
 ****************************************/
/**
 * mpu6050ConfigI2C1
 * @note I2C1 | SCL : PB6, SDA : PB7, Speed : 400k
 * @return mpu6050_t*
 */
red_mpu6050Port_t* redMpu6050Init(uint8_t mpu6050PortNum, red_mpu6050_userSetting_t* userSetting) {
  assert_param(IS_VAILD_MPU6050_PORT_NUM(mpu6050PortNum));

  red_mpu6050Port_t* mpu6050Port = &mpu6050Ports[mpu6050PortNum];

  red_i2cDevice_t* mpu6050I2c = redI2cInit(userSetting->i2cDeviceNum, userSetting->i2cUserSetting);

  mpu6050Settings[mpu6050PortNum].i2cDevice = mpu6050I2c;
  mpu6050Settings[mpu6050PortNum].userSetting = userSetting;
  mpu6050Port->param = &mpu6050Settings[mpu6050PortNum];

  mpu6050Port->gyroInit = red_mpu6050GyroInit;
  mpu6050Port->gyroDetect = red_mpu6050GyroDetect;
  mpu6050Port->gyroRead = red_mpu6050GyroRead;
  mpu6050Port->gyroScaleFactor = red_mpu6050GyroScalefactor;

  mpu6050Port->accInit = red_mpu6050AccInit;
  mpu6050Port->accDetect = red_mpu6050AccDetect;
  mpu6050Port->accRead = red_mpu6050AccRead;
  mpu6050Port->accScaleFactor = red_mpu6050AccScalefactor;

  mpu6050Port->tempRead = red_mpu6050TempRead;

  red_mpu6050Config(mpu6050Port);
  return mpu6050Port;
}
