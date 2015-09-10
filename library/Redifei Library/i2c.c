/*
 i2c.c - i2c library
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

#include <stdlib.h>
#include <stdbool.h>
#include <stm32f10x_conf.h>
#include "i2c.h"

#define I2C_DEFAULT_TIMEOUT 30000

#define BUF_SIZE 32

typedef struct {
  uint8_t buf[BUF_SIZE];
} red_i2cPortBuf_t;

static red_i2cPort_t i2cPorts[RED_I2C_PORT_MAX];
static red_i2c_setting_t i2cSettings[RED_I2C_PORT_MAX];
static red_i2cPortBuf_t i2cPortBuf[RED_I2C_PORT_MAX];

// FIXME: Untested
static void i2cUnstick(struct red_i2cPort* this) {
#if 0
  const red_i2c_hardware_t* i2cHW = this->setting->hw;
  red_i2c_userSetting_t* i2cUserSetting = this->setting->userSetting;

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = i2cHW->scl_gpioPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(i2cHW->scl_gpioPort, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = i2cHW->sda_gpioPin;
  GPIO_Init(i2cHW->sda_gpioPort, &GPIO_InitStructure);

  GPIO_WriteBit(i2cHW->scl_gpioPort, i2cHW->scl_gpioPin, Bit_SET);
  GPIO_WriteBit(i2cHW->sda_gpioPort, i2cHW->sda_gpioPin, Bit_SET);

  uint8_t i;
  for (i = 0; i < 8; i++) {
    // Wait for any clock stretching to finish
    while (!GPIO_ReadInputDataBit(i2cHW->scl_gpioPort, i2cHW->scl_gpioPin))
    delayMicroseconds(10);

    // Pull low
    GPIO_WriteBit(i2cHW->scl_gpioPort, i2cHW->scl_gpioPin, Bit_RESET);// Set bus low
    delayMicroseconds(10);
    // Release high again
    GPIO_WriteBit(i2cHW->scl_gpioPort, i2cHW->scl_gpioPin, Bit_SET);// Set bus high
    delayMicroseconds(10);
  }

  GPIO_WriteBit(i2cHW->sda_gpioPort, i2cHW->sda_gpioPin, Bit_RESET); // Set bus data low
  delayMicroseconds(10);
  GPIO_WriteBit(i2cHW->scl_gpioPort, i2cHW->scl_gpioPin, Bit_RESET);// Set bus scl low
  delayMicroseconds(10);
  GPIO_WriteBit(i2cHW->scl_gpioPort, i2cHW->scl_gpioPin, Bit_SET);// Set bus scl high
  delayMicroseconds(10);
  GPIO_WriteBit(i2cHW->sda_gpioPort, i2cHW->sda_gpioPin, Bit_SET);// Set bus sda high
#endif
}

static void red_i2cConfig(struct red_i2cPort* this) {
  const red_i2c_hardware_t* i2cHW = this->setting->hw;
  red_i2c_userSetting_t* i2cUserSetting = this->setting->userSetting;

  RCC_APB2PeriphClockCmd(i2cHW->scl_gpioClock, ENABLE);
  RCC_APB2PeriphClockCmd(i2cHW->sda_gpioClock, ENABLE);
  RCC_APB1PeriphClockCmd(i2cHW->i2cClock, ENABLE);

  // clock out stuff to make sure slaves arent stuck
  i2cUnstick(this);

  // Init pins
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = i2cHW->scl_gpioPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(i2cHW->scl_gpioPort, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = i2cHW->sda_gpioPin;
  GPIO_Init(i2cHW->sda_gpioPort, &GPIO_InitStructure);

  // Init I2C
  I2C_DeInit(i2cHW->i2cPort);

  I2C_InitTypeDef I2C_InitStructure;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = i2cUserSetting->clockSpeed;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_OwnAddress1 = 0; // Slave Only
  I2C_Init(i2cHW->i2cPort, &I2C_InitStructure);

  if (i2cUserSetting->i2cMode == RED_I2C_INTERRPUT_MODE) {
    //Enable EVT and ERR interrupts - they are enabled by the first request
    I2C_ITConfig(i2cHW->i2cPort, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    // I2C ER Interrupt
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = i2cHW->i2cErIRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // I2C EV Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = i2cHW->i2cEvIRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
  }

  I2C_Cmd(i2cHW->i2cPort, ENABLE);
}

static uint8_t red_i2cWriteBytes(struct red_i2cPort* this, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
  const red_i2c_hardware_t* i2cHW = this->setting->hw;
  red_i2c_userSetting_t* i2cUserSetting = this->setting->userSetting;

  uint32_t timeout = I2C_DEFAULT_TIMEOUT;
  I2C_TypeDef* I2Cx = i2cHW->i2cPort;

  if (i2cUserSetting->i2cMode == RED_I2C_INTERRPUT_MODE) {
    this->setting->i2cDirection = I2C_Direction_Transmitter;
    this->setting->deviceIDSent = false;

    this->setting->queue.tail = this->setting->queue.head;

    this->setting->queue.buf[this->setting->queue.head++] = addr << 1;
    this->setting->queue.head %= this->setting->queue.size;
    this->setting->queue.buf[this->setting->queue.head++] = reg;
    this->setting->queue.head %= this->setting->queue.size;
    this->setting->i2cLength = len;

    for (uint8_t i = 0; i < len; i++) {
      this->setting->queue.buf[this->setting->queue.head++] = data[i];
      this->setting->queue.head %= this->setting->queue.size;
    }

    this->setting->busy = true;
    this->setting->error = 0;

    if ((I2Cx->CR2 & I2C_IT_EVT) == RESET) {
      //START를 확실하게 보낸다
      if ((I2Cx->CR1 & I2C_CR1_START) == RESET) {
        //전송이 끝나고 STOP를 기다린다
        while ((I2Cx->CR1 & I2C_CR1_STOP) == SET)
          ;
        //새로운 작업의 START를 전송한다
        I2C_GenerateSTART(I2Cx, ENABLE);
      }
      // 인터럽트를 재시작 하도록 허용
      I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
    }

    while (this->setting->busy && --timeout > 0)
      ;
    if (timeout == 0) {
      this->setting->errCount++;
      // reinit peripheral + clock out garbage
      red_i2cConfig(this);
      return 1;
    }
  }
  else if (i2cUserSetting->i2cMode == RED_I2C_POLLING_MODE) {
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    I2C_GenerateSTART(I2Cx, ENABLE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
      ;

    I2C_Send7bitAddress(I2Cx, addr << 1, I2C_Direction_Transmitter); // SLA+W
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
      ;

    I2C_SendData(I2Cx, reg);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
      ;

    I2C_SendData(I2Cx, *data);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
      ;

    I2C_GenerateSTOP(I2Cx, ENABLE);
  }

  return this->setting->error;
}

static uint8_t red_i2cWrite1Byte(struct red_i2cPort* this, uint8_t addr, uint8_t reg, uint8_t data) {
  return red_i2cWriteBytes(this, addr, reg, 1, &data);
}

static uint8_t red_i2cReadBytes(struct red_i2cPort* this, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf) {
  const red_i2c_hardware_t* i2cHW = this->setting->hw;
  red_i2c_userSetting_t* i2cUserSetting = this->setting->userSetting;

  uint32_t timeout = 30000;
  I2C_TypeDef* I2Cx = i2cHW->i2cPort;

  if (i2cUserSetting->i2cMode == RED_I2C_INTERRPUT_MODE) {
    this->setting->i2cDirection = I2C_Direction_Receiver;
    this->setting->deviceIDSent = false;

    this->setting->queue.tail = this->setting->queue.head;
    this->setting->queue.buf[this->setting->queue.head++] = addr << 1; // SLA+W
    this->setting->queue.head %= this->setting->queue.size;
    this->setting->queue.buf[this->setting->queue.head++] = reg; // REG
    this->setting->queue.head %= this->setting->queue.size;
    this->setting->queue.buf[this->setting->queue.head++] = addr << 1; // SLA+R
    this->setting->queue.head %= this->setting->queue.size;
    this->setting->i2cLength = len;

    this->setting->busy = true;
    this->setting->error = 0;

    if ((I2Cx->CR2 & I2C_IT_EVT) == RESET) {
      //START를 확실하게 보낸다
      if ((I2Cx->CR1 & I2C_CR1_START) == RESET) {
        //전송이 끝나고 STOP를 기다린다
        while ((I2Cx->CR1 & I2C_CR1_STOP) == SET)
          ;
        //새로운 작업의 START를 전송한다
        I2C_GenerateSTART(I2Cx, ENABLE);
      }
      // 인터럽트를 재시작 하도록 허용
      I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
    }

    while (this->setting->busy && --timeout > 0)
      ;
    if (timeout == 0) {
      this->setting->errCount++;
      // reinit peripheral + clock out garbage
      red_i2cConfig(this);
//      return 1; // FIXME: Why not reset busy bit!!
    }

    for (uint8_t i = 0; i < len; i++) {
      if (this->setting->queue.tail != this->setting->queue.head) {
        buf[i] = this->setting->queue.buf[this->setting->queue.tail++];
        this->setting->queue.tail %= this->setting->queue.size;
      }
      else
        return 2;
    }
  }
  else if (i2cUserSetting->i2cMode == RED_I2C_POLLING_MODE) {
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    I2C_GenerateSTART(I2Cx, ENABLE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
      ;

    I2C_Send7bitAddress(I2Cx, addr << 1, I2C_Direction_Transmitter); // SLA+W
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
      ;

    I2C_SendData(I2Cx, reg);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
      ;

    I2C_GenerateSTART(I2Cx, ENABLE); // Rep Start
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
      ;

    I2C_Send7bitAddress(I2Cx, addr << 1, I2C_Direction_Receiver); // SLA+R
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
      ;

    uint8_t i = 0;
    for (i = 0; i < len; i++) {
      if (i == len - 1) {
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        I2C_GenerateSTOP(I2Cx, ENABLE);
      }

      while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        ;
      *(buf + i) = I2C_ReceiveData(I2Cx);
    }

    I2C_AcknowledgeConfig(I2Cx, ENABLE);
  }

  return this->setting->error;
}

static uint8_t red_i2cRead1Byte(struct red_i2cPort* this, uint8_t addr, uint8_t reg, uint8_t* buf) {
  return red_i2cReadBytes(this, addr, reg, 1, buf);
}

red_i2cPort_t* redI2cInit(uint8_t i2cPortNum, red_i2c_userSetting_t* userSetting) {
  red_i2cPort_t* i2cPort = &i2cPorts[i2cPortNum];

  i2cSettings[i2cPortNum].hw = &redI2cHardWareMap[i2cPortNum];
  i2cSettings[i2cPortNum].userSetting = userSetting;
  i2cPort->setting = &i2cSettings[i2cPortNum];

  i2cPort->setting->queue.buf = i2cPortBuf[i2cPortNum].buf;
  i2cPort->setting->queue.size = BUF_SIZE;

  i2cPort->setting->error = 0;

  i2cPort->write1Byte = red_i2cWrite1Byte;
  i2cPort->writeBytes = red_i2cWriteBytes;
  i2cPort->read1Byte = red_i2cRead1Byte;
  i2cPort->readBytes = red_i2cReadBytes;

  red_i2cConfig(i2cPort);
  return i2cPort;
}

static void red_i2cER_handler(struct red_i2cPort* this) {
  const red_i2c_hardware_t* i2cHW = this->setting->hw;
  red_i2c_userSetting_t* i2cUserSetting = this->setting->userSetting;
  I2C_TypeDef* I2Cx = i2cHW->i2cPort;

  volatile uint32_t SR1Register = I2Cx->SR1;

  // 0x0F00 => OVR, AF, ARLO, BERR
  if (SR1Register & 0x0F00)
    this->setting->error = -1;

  // 0x0700 => AF, ARLO, BERR
  if (SR1Register & 0x0700) {
    (void) I2Cx->SR2;
    I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);

    // 0x0200 => ARLO
    if (!(SR1Register & 0x0200) && ((I2Cx->CR1 & I2C_CR1_STOP) == RESET)) {
      if (I2Cx->CR1 & I2C_CR1_START) {
        while ((I2Cx->CR1 & I2C_CR1_START) != RESET)
          ;
        I2C_GenerateSTOP(I2Cx, ENABLE);
        while ((I2Cx->CR1 & I2C_CR1_STOP) != RESET)
          ;
        red_i2cConfig(this);
      }
      else {
        I2C_GenerateSTOP(I2Cx, ENABLE);
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
      }
    }
  }
  // 0x0F00 => OVR, AF, ARLO, BERR
  I2Cx->SR1 &= ~0x0F00;
  this->setting->busy = false;
}

static void red_i2cEV_handler(struct red_i2cPort* this) {
  const red_i2c_hardware_t* i2cHW = this->setting->hw;
  red_i2c_userSetting_t* i2cUserSetting = this->setting->userSetting;
  uint8_t subaddress_sent = this->setting->deviceIDSent;
  static uint8_t final_stop;

  uint8_t end = false;

  I2C_TypeDef* I2Cx = i2cHW->i2cPort;
  uint8_t bytes = this->setting->i2cLength;

  uint8_t writing = false, reading = false;
  (this->setting->i2cDirection == I2C_Direction_Transmitter) ? (writing = true) : (reading = true);

  uint8_t SReg_1 = I2Cx->SR1;

  /* START */
  if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) != RESET) {
    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    if (reading && subaddress_sent) {
      this->setting->deviceIDSent = true;
      if (bytes == 2)
        I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);
      I2C_Send7bitAddress(I2Cx, this->setting->queue.buf[this->setting->queue.tail++], I2C_Direction_Receiver);
    }
    else
      I2C_Send7bitAddress(I2Cx, this->setting->queue.buf[this->setting->queue.tail++], I2C_Direction_Transmitter);
    this->setting->queue.tail %= this->setting->queue.size;
  }

  /* ADDR */
  else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR) != RESET) {
    __DMB();
    if (bytes == 1 && reading && subaddress_sent) {
      I2C_AcknowledgeConfig(I2Cx, DISABLE);
      __DMB();
      (void) I2Cx->SR2;
      I2C_GenerateSTOP(I2Cx, ENABLE);
      final_stop = 1;
      I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
    }
    else {
      (void) I2Cx->SR2;
      __DMB();
      if (bytes == 2 && reading && subaddress_sent) {
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
      }
      else if (bytes == 3 && reading && subaddress_sent)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
      else
        I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
    }
  }

  /* BTF */
  else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF) != RESET) {
    final_stop = 1;

    if (reading && subaddress_sent) {
      if (bytes > 2) {
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        this->setting->queue.buf[this->setting->queue.head++] = I2C_ReceiveData(I2Cx);
        this->setting->queue.head %= this->setting->queue.size;
        I2C_GenerateSTOP(I2Cx, ENABLE);
        final_stop = 1;
        this->setting->queue.buf[this->setting->queue.head++] = I2C_ReceiveData(I2Cx);
        this->setting->queue.head %= this->setting->queue.size;
        I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
      }
      else {
        if (final_stop)
          I2C_GenerateSTOP(I2Cx, ENABLE);
        else
          I2C_GenerateSTART(I2Cx, ENABLE);
        this->setting->queue.buf[this->setting->queue.head++] = I2C_ReceiveData(I2Cx);
        this->setting->queue.head %= this->setting->queue.size;
        this->setting->queue.buf[this->setting->queue.head++] = I2C_ReceiveData(I2Cx);
        this->setting->queue.head %= this->setting->queue.size;
        end = true;
      }
    }
    else {
      if (subaddress_sent || (writing)) {
        if (final_stop)
          I2C_GenerateSTOP(I2Cx, ENABLE);
        else
          I2C_GenerateSTART(I2Cx, ENABLE);
        end = true;
      }
      else {
        I2C_GenerateSTART(I2Cx, ENABLE);
        this->setting->deviceIDSent = true; // Read일땐, REP START후에 설정
      }
    }
    while ((I2Cx->CR1 & I2C_CR1_START) != RESET)
      ;
  }

  /* RXNE */
  else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE) != RESET) {
    this->setting->queue.buf[this->setting->queue.head++] = I2C_ReceiveData(I2Cx);
    this->setting->queue.head %= this->setting->queue.size;
    uint8_t index = this->setting->i2cLength
        - (((this->setting->queue.head + this->setting->queue.size) - this->setting->queue.tail) % this->setting->queue.size);
    if (bytes == (index + 3))
      I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
    if (bytes == index)
      end = true;
  }

  /* TXE */
  else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) != RESET) {
    if (subaddress_sent != false) {
      I2C_SendData(I2Cx, this->setting->queue.buf[this->setting->queue.tail++]);
      this->setting->queue.tail %= this->setting->queue.size;
      uint8_t index = this->setting->i2cLength
          - (((this->setting->queue.head + this->setting->queue.size) - this->setting->queue.tail) % this->setting->queue.size);
      if (bytes == index)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
    }
    else {
      I2C_SendData(I2Cx, this->setting->queue.buf[this->setting->queue.tail++]); //reg
      this->setting->queue.tail %= this->setting->queue.size;
      if (writing)
        this->setting->deviceIDSent = true; // 추가된 부분/ Write는 REP START가 없으므로
      if (reading || !bytes)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
    }
  }

  /* END */
  if (end != false) {
    this->setting->deviceIDSent = false;
    if (final_stop)
      I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    this->setting->busy = false;
  }
}

void I2C1_ER_IRQHandler(void) {
  red_i2cER_handler(&i2cPorts[RED_I2C_PORT_1]);
}

void I2C1_EV_IRQHandler(void) {
  red_i2cEV_handler(&i2cPorts[RED_I2C_PORT_1]);
}

void I2C2_ER_IRQHandler(void) {
  red_i2cER_handler(&i2cPorts[RED_I2C_PORT_2]);
}

void I2C2_EV_IRQHandler(void) {
  red_i2cEV_handler(&i2cPorts[RED_I2C_PORT_2]);
}

