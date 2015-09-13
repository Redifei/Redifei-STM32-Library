/*
 i2c.h - i2c library
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

#include "queue.h"

enum {
  RED_I2C_PORT_1,
  RED_I2C_PORT_2,
  RED_I2C_PORT_MAX,
};

typedef enum {
  RED_I2C_INTERRPUT_MODE, RED_I2C_POLLING_MODE, RED_I2C_SOFTWARE_MODE,
} red_i2cMode_t;

enum {
  I2C_DIRECTION_TX = 0,
  I2C_DIRECTION_RX,
};

typedef struct {
  GPIO_TypeDef* sda_gpioPort;
  uint32_t sda_gpioClock;
  uint16_t sda_gpioPin;

  GPIO_TypeDef* scl_gpioPort;
  uint32_t scl_gpioClock;
  uint16_t scl_gpioPin;

  I2C_TypeDef* i2cPort;
  uint32_t i2cClock;

  IRQn_Type i2cEvIRQ;
  IRQn_Type i2cErIRQ;
} red_i2c_hardware_t;

static const red_i2c_hardware_t redI2cHardWareMap[RED_I2C_PORT_MAX] = {
    { GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_7, GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_6, I2C1, RCC_APB1Periph_I2C1, I2C1_EV_IRQn, I2C1_ER_IRQn },
    { GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_11, GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_10, I2C2, RCC_APB1Periph_I2C2, I2C2_EV_IRQn, I2C2_ER_IRQn },
};

typedef struct {
  red_i2cMode_t i2cMode;
  uint32_t clockSpeed;
  uint32_t timeOut;
//  uint16_t bufSize;
} red_i2c_userSetting_t;

typedef struct {
  const red_i2c_hardware_t* hw;
  red_i2c_userSetting_t* userSetting;

  uint8_t I2CDirection;
  uint8_t Address;
  uint16_t NumbOfBytes;

  uint8_t index;
  uint8_t* buffer;

  uint8_t finish; // TODO
  uint8_t generateStop; // TODO
} red_i2c_setting_t;

typedef struct red_i2cPort {
  red_i2c_setting_t* setting;
  bool (*reset)(struct red_i2cPort* this);
  bool (*readBytes)(struct red_i2cPort* this, uint8_t SlaveAddress, uint8_t NumByteToRead, uint8_t* pBuffer);
  bool (*read1Byte)(struct red_i2cPort* this, uint8_t SlaveAddress, uint8_t* pBuffer);
  bool (*writeBytes)(struct red_i2cPort* this, uint8_t SlaveAddress, uint8_t NumByteToWrite, uint8_t* pBuffer);
  bool (*write1Byte)(struct red_i2cPort* this, uint8_t SlaveAddress, uint8_t buffer);
} red_i2cPort_t;

#define I2C_DEFAULT_TIMEOUT 30000

#define IS_CONFIGED_I2C_PORT(THIS_SETTING) (THIS_SETTING != NULL)
#define IS_VAILD_I2C_PORT_NUM(I2C_NUM) (I2C_NUM < RED_I2C_PORT_MAX)

red_i2cPort_t* redI2cInit(uint8_t i2cPortNum, red_i2c_userSetting_t* userSetting);
