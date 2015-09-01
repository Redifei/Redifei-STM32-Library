/*
 * Redifei: I2c Port Library
 * Based on BaseFlight Project
 *
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 *
 *
 * TODO(I2C):
 * Support softWare mode
 * Support write bytes function
 * Done : Independent queue type
 * FIXME(I2C):
 * Error busy flag in i2cReadBytes Function
 * Support i2cUnstick
 *
 *
 * -Functions
 * openI2Cx : x(1;2)
 * ->writeBytes
 * ->write1Byte
 * ->readBytes
 * ->read1Byte
 */

#pragma once

#include "queue.h"

typedef uint8_t bool;
#define false (bool)0
#define true  (bool)1

#define I2C_DEFAULT_TIMEOUT 30000

#define I2C_CR1_START ((uint16_t)0x0100)
#define I2C_CR1_STOP ((uint16_t)0x0200)

typedef enum {
  I2C_INTERRPUT_MODE, I2C_POLLING_MODE, I2C_SOFTWARE_MODE,
} i2cMode_t;

typedef struct {
  /*하드웨어 기본 설정*/
  uint16_t sda_gpioPin;
  GPIO_TypeDef* sda_gpioPort;
  uint32_t sda_gpioClock;

  uint16_t scl_gpioPin;
  GPIO_TypeDef* scl_gpioPort;
  uint32_t scl_gpioClock;

  /*아이스퀘어씨 포트 기본 설정*/
  I2C_TypeDef* i2cPort;
  uint32_t i2cClock;

  IRQn_Type i2cEvIRQ;
  IRQn_Type i2cErIRQ;

  /*아이스퀘어씨 포트 추가 설정*/
  uint32_t i2cSpeed;

  uint8_t i2cDirection; // I2C_Direction_Transmitter
  uint8_t i2cLength;

  uint8_t busy;
  uint8_t error;

  uint8_t deviceIDSent; // In IRQ

  uint16_t errCount;

  /*아이스퀘어씨 통신에서 사용할 큐*/
  Qtype_t queue;

  /*아이스퀘어씨 포트에서 사용할 함수*/
  uint8_t (*writeBytes)(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data);
  /**
   * Write1Byte
   * @param addr : Device Address
   * @param reg : Register
   * @param data : Register Data
   * @return 0 : success
   */
  uint8_t (*write1Byte)(uint8_t addr, uint8_t reg, uint8_t data);
  /**
   * ReadBytes
   * @param addr : Device Address
   * @param reg : Register
   * @param len : Buffer Length
   * @param buf : Buffer
   * @return 0 : success
   */
  uint8_t (*readBytes)(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
  /**
   * @param addr : Device Address
   * @param reg : Register
   * @param buf : Buffer
   * @return 0 : success
   */
  uint8_t (*read1Byte)(uint8_t addr, uint8_t reg, uint8_t* buf);

  i2cMode_t i2cMode;
} i2cPort_t;

i2cPort_t* openI2C1(i2cMode_t mode);
i2cPort_t* openI2C2(i2cMode_t mode);
