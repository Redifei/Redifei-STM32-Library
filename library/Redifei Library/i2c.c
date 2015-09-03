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
 * (Checking) Support i2cUnstick
 *
 *
 * -Functions
 * openI2Cx : x(1;2)
 * ->writeBytes
 * ->write1Byte
 * ->readBytes
 * ->read1Byte
 */

#include <stdbool.h>
#include <stm32f10x_conf.h>
#include "systickTimer.h"
#include "i2c.h"

#define I2C_DEFAULT_TIMEOUT 30000

#define BUF_SIZE 32

static i2cPort_t i2cPort1;
static i2cPort_t i2cPort2;

static uint8_t i2cPort1Buf[BUF_SIZE];
static uint8_t i2cPort2Buf[BUF_SIZE];

static uint8_t I2C1_writeBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data);
static uint8_t I2C1_write1Byte(uint8_t addr, uint8_t reg, uint8_t data);
static uint8_t I2C1_readBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
static uint8_t I2C1_read1Byte(uint8_t addr, uint8_t reg, uint8_t* buf);

static uint8_t I2C2_writeBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data);
static uint8_t I2C2_write1Byte(uint8_t addr, uint8_t reg, uint8_t data);
static uint8_t I2C2_readBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
static uint8_t I2C2_read1Byte(uint8_t addr, uint8_t reg, uint8_t* buf);

static void i2cOpen(i2cPort_t* instance);

static uint8_t i2cWriteBytes(i2cPort_t* instance, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
  uint32_t timeout = I2C_DEFAULT_TIMEOUT;
  I2C_TypeDef* I2Cx = instance->i2cPort;

  if (instance->i2cMode == I2C_INTERRPUT_MODE) {
    instance->i2cDirection = I2C_Direction_Transmitter;
    instance->deviceIDSent = false;

    instance->queue.tail = instance->queue.head;

    instance->queue.buf[instance->queue.head++] = addr << 1;
    instance->queue.head %= instance->queue.size;
    instance->queue.buf[instance->queue.head++] = reg;
    instance->queue.head %= instance->queue.size;
    instance->i2cLength = len;

    for (uint8_t i = 0; i < len; i++) {
      instance->queue.buf[instance->queue.head++] = data[i];
      instance->queue.head %= instance->queue.size;
    }

    instance->busy = true;
    instance->error = 0;

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

    while (instance->busy && --timeout > 0)
      ;
    if (timeout == 0) {
      instance->errCount++;
      // reinit peripheral + clock out garbage
      i2cOpen(instance);
      return 1;
    }
  }
  else if (instance->i2cMode == I2C_POLLING_MODE) {
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

  return instance->error;
}

static uint8_t i2cReadBytes(i2cPort_t* instance, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf) {
  uint32_t timeout = 30000;
  I2C_TypeDef* I2Cx = instance->i2cPort;

  if (instance->i2cMode == I2C_INTERRPUT_MODE) {
    instance->i2cDirection = I2C_Direction_Receiver;
    instance->deviceIDSent = false;

    instance->queue.tail = instance->queue.head;
    instance->queue.buf[instance->queue.head++] = addr << 1; // SLA+W
    instance->queue.head %= instance->queue.size;
    instance->queue.buf[instance->queue.head++] = reg; // REG
    instance->queue.head %= instance->queue.size;
    instance->queue.buf[instance->queue.head++] = addr << 1; // SLA+R
    instance->queue.head %= instance->queue.size;
    instance->i2cLength = len;

    instance->busy = true;
    instance->error = 0;

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

    while (instance->busy && --timeout > 0)
      ;
    if (timeout == 0) {
      instance->errCount++;
      // reinit peripheral + clock out garbage
      i2cOpen(instance);
//      return 1; // FIXME: Why not reset busy bit!!
    }

    for (uint8_t i = 0; i < len; i++) {
      if (instance->queue.tail != instance->queue.head) {
        buf[i] = instance->queue.buf[instance->queue.tail++];
        instance->queue.tail %= instance->queue.size;
      }
      else
        return 2;
    }
  }
  else if (instance->i2cMode == I2C_POLLING_MODE) {
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

  return instance->error;
}

// FIXME: Untested
static void i2cUnstick(i2cPort_t* instance) {
#if 0
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = instance->scl_gpioPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(instance->scl_gpioPort, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = instance->sda_gpioPin;
  GPIO_Init(instance->sda_gpioPort, &GPIO_InitStructure);

  GPIO_WriteBit(instance->scl_gpioPort, instance->scl_gpioPin, Bit_SET);
  GPIO_WriteBit(instance->sda_gpioPort, instance->sda_gpioPin, Bit_SET);

  uint8_t i;
  for (i = 0; i < 8; i++) {
    // Wait for any clock stretching to finish
    while (!GPIO_ReadInputDataBit(instance->scl_gpioPort, instance->scl_gpioPin))
    delayMicroseconds(10);

    // Pull low
    GPIO_WriteBit(instance->scl_gpioPort, instance->scl_gpioPin, Bit_RESET);// Set bus low
    delayMicroseconds(10);
    // Release high again
    GPIO_WriteBit(instance->scl_gpioPort, instance->scl_gpioPin, Bit_SET);// Set bus high
    delayMicroseconds(10);
  }

  GPIO_WriteBit(instance->sda_gpioPort, instance->sda_gpioPin, Bit_RESET); // Set bus data low
  delayMicroseconds(10);
  GPIO_WriteBit(instance->scl_gpioPort, instance->scl_gpioPin, Bit_RESET);// Set bus scl low
  delayMicroseconds(10);
  GPIO_WriteBit(instance->scl_gpioPort, instance->scl_gpioPin, Bit_SET);// Set bus scl high
  delayMicroseconds(10);
  GPIO_WriteBit(instance->sda_gpioPort, instance->sda_gpioPin, Bit_SET);// Set bus sda high
#endif
}

static void i2cOpen(i2cPort_t* instance) {
  RCC_APB2PeriphClockCmd(instance->scl_gpioClock, ENABLE);
  RCC_APB2PeriphClockCmd(instance->sda_gpioClock, ENABLE);
  RCC_APB1PeriphClockCmd(instance->i2cClock, ENABLE);

  // clock out stuff to make sure slaves arent stuck
  i2cUnstick(instance);

  // Init pins
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = instance->scl_gpioPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(instance->scl_gpioPort, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = instance->sda_gpioPin;
  GPIO_Init(instance->sda_gpioPort, &GPIO_InitStructure);

  // Init I2C
  I2C_DeInit(instance->i2cPort);

  I2C_InitTypeDef I2C_InitStructure;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = instance->i2cSpeed;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_OwnAddress1 = 0; // Slave Only
  I2C_Init(instance->i2cPort, &I2C_InitStructure);

  if (instance->i2cMode == I2C_INTERRPUT_MODE) {
    //Enable EVT and ERR interrupts - they are enabled by the first request
    I2C_ITConfig(instance->i2cPort, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    // I2C ER Interrupt
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = instance->i2cErIRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // I2C EV Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = instance->i2cEvIRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
  }

  I2C_Cmd(instance->i2cPort, ENABLE);
}

/**
 * openI2C1
 * @note SCL : PB6, SDA : PB7, Speed : 400k
 * @param mode : I2C_INTERRPUT_MODE,
 I2C_POLLING_MODE,
 I2C_SOFTWARE_MODE
 * @return i2cPort_t*
 */
i2cPort_t* openI2C1(i2cMode_t mode) {
  i2cPort1.sda_gpioClock = RCC_APB2Periph_GPIOB;
  i2cPort1.sda_gpioPin = GPIO_Pin_7;
  i2cPort1.sda_gpioPort = GPIOB;

  i2cPort1.scl_gpioClock = RCC_APB2Periph_GPIOB;
  i2cPort1.scl_gpioPin = GPIO_Pin_6;
  i2cPort1.scl_gpioPort = GPIOB;

  i2cPort1.i2cClock = RCC_APB1Periph_I2C1;
  i2cPort1.i2cPort = I2C1;
  i2cPort1.i2cEvIRQ = I2C1_EV_IRQn;
  i2cPort1.i2cErIRQ = I2C1_ER_IRQn;

  i2cPort1.i2cSpeed = 400000;

  i2cPort1.writeBytes = I2C1_writeBytes;
  i2cPort1.write1Byte = I2C1_write1Byte;
  i2cPort1.readBytes = I2C1_readBytes;
  i2cPort1.read1Byte = I2C1_read1Byte;

  i2cPort1.i2cMode = mode;

  i2cPort1.queue.buf = i2cPort1Buf;
  i2cPort1.queue.size = BUF_SIZE;

  i2cPort1.errCount = 0;
  i2cOpen(&i2cPort1);

  return &i2cPort1;
}

static uint8_t I2C1_writeBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data) {
  return i2cWriteBytes(&i2cPort1, addr, reg, len, data);
}

static uint8_t I2C1_write1Byte(uint8_t addr, uint8_t reg, uint8_t data) {
  return i2cWriteBytes(&i2cPort1, addr, reg, 1, &data);
}

static uint8_t I2C1_readBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf) {
  return i2cReadBytes(&i2cPort1, addr, reg, len, buf);
}

static uint8_t I2C1_read1Byte(uint8_t addr, uint8_t reg, uint8_t* buf) {
  return i2cReadBytes(&i2cPort1, addr, reg, 1, buf);
}

/**
 * openI2C2
 * @note SCL : PB10, SDA : PB11, Speed : 400k
 * @param mode : I2C_INTERRPUT_MODE,
 I2C_POLLING_MODE,
 I2C_SOFTWARE_MODE
 * @return i2cPort_t*
 */
i2cPort_t* openI2C2(i2cMode_t mode) {
  i2cPort2.sda_gpioClock = RCC_APB2Periph_GPIOB;
  i2cPort2.sda_gpioPin = GPIO_Pin_11;
  i2cPort2.sda_gpioPort = GPIOB;

  i2cPort2.scl_gpioClock = RCC_APB2Periph_GPIOB;
  i2cPort2.scl_gpioPin = GPIO_Pin_10;
  i2cPort2.scl_gpioPort = GPIOB;

  i2cPort2.i2cClock = RCC_APB1Periph_I2C2;
  i2cPort2.i2cPort = I2C2;
  i2cPort2.i2cEvIRQ = I2C2_EV_IRQn;
  i2cPort2.i2cErIRQ = I2C2_ER_IRQn;

  i2cPort2.i2cSpeed = 400000;

  i2cPort2.writeBytes = I2C2_writeBytes;
  i2cPort2.write1Byte = I2C2_write1Byte;
  i2cPort2.readBytes = I2C2_readBytes;
  i2cPort2.read1Byte = I2C2_read1Byte;

  i2cPort2.i2cMode = mode;

  i2cPort2.queue.buf = i2cPort2Buf;
  i2cPort2.queue.size = BUF_SIZE;

  i2cPort2.errCount = 0;
  i2cOpen(&i2cPort2);

  return &i2cPort2;
}

static uint8_t I2C2_writeBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data) {
  return i2cWriteBytes(&i2cPort2, addr, reg, len, data);
}

static uint8_t I2C2_write1Byte(uint8_t addr, uint8_t reg, uint8_t data) {
  return i2cWriteBytes(&i2cPort2, addr, reg, 1, &data);
}

static uint8_t I2C2_readBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf) {
  return i2cReadBytes(&i2cPort2, addr, reg, len, buf);
}

static uint8_t I2C2_read1Byte(uint8_t addr, uint8_t reg, uint8_t* buf) {
  return i2cReadBytes(&i2cPort2, addr, reg, 1, buf);
}

static void i2c_er_handler(i2cPort_t* instance) {
  I2C_TypeDef* I2Cx = instance->i2cPort;

  volatile uint32_t SR1Register = I2Cx->SR1;

  // 0x0F00 => OVR, AF, ARLO, BERR
  if (SR1Register & 0x0F00)
    instance->error = -1;

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
        i2cOpen(instance);
      }
      else {
        I2C_GenerateSTOP(I2Cx, ENABLE);
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
      }
    }
  }
  // 0x0F00 => OVR, AF, ARLO, BERR
  I2Cx->SR1 &= ~0x0F00;
  instance->busy = false;
}

static void i2c_ev_handler(i2cPort_t* instance) {
  uint8_t subaddress_sent = instance->deviceIDSent;
  static uint8_t final_stop;

  uint8_t end = false;

  I2C_TypeDef* I2Cx = instance->i2cPort;
  uint8_t bytes = instance->i2cLength;

  uint8_t writing = false, reading = false;
  (instance->i2cDirection == I2C_Direction_Transmitter) ? (writing = true) : (reading = true);

  uint8_t SReg_1 = I2Cx->SR1;

  /* START */
  if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) != RESET) {
    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    if (reading && subaddress_sent) {
      instance->deviceIDSent = true;
      if (bytes == 2)
        I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);
      I2C_Send7bitAddress(I2Cx, instance->queue.buf[instance->queue.tail++], I2C_Direction_Receiver);
    }
    else
      I2C_Send7bitAddress(I2Cx, instance->queue.buf[instance->queue.tail++], I2C_Direction_Transmitter);
    instance->queue.tail %= instance->queue.size;
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
        instance->queue.buf[instance->queue.head++] = I2C_ReceiveData(I2Cx);
        instance->queue.head %= instance->queue.size;
        I2C_GenerateSTOP(I2Cx, ENABLE);
        final_stop = 1;
        instance->queue.buf[instance->queue.head++] = I2C_ReceiveData(I2Cx);
        instance->queue.head %= instance->queue.size;
        I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
      }
      else {
        if (final_stop)
          I2C_GenerateSTOP(I2Cx, ENABLE);
        else
          I2C_GenerateSTART(I2Cx, ENABLE);
        instance->queue.buf[instance->queue.head++] = I2C_ReceiveData(I2Cx);
        instance->queue.head %= instance->queue.size;
        instance->queue.buf[instance->queue.head++] = I2C_ReceiveData(I2Cx);
        instance->queue.head %= instance->queue.size;
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
        instance->deviceIDSent = true; // Read일땐, REP START후에 설정
      }
    }
    while ((I2Cx->CR1 & I2C_CR1_START) != RESET)
      ;
  }

  /* RXNE */
  else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE) != RESET) {
    instance->queue.buf[instance->queue.head++] = I2C_ReceiveData(I2Cx);
    instance->queue.head %= instance->queue.size;
    uint8_t index = instance->i2cLength
        - (((instance->queue.head + instance->queue.size) - instance->queue.tail) % instance->queue.size);
    if (bytes == (index + 3))
      I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
    if (bytes == index)
      end = true;
  }

  /* TXE */
  else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) != RESET) {
    if (subaddress_sent != false) {
      I2C_SendData(I2Cx, instance->queue.buf[instance->queue.tail++]);
      instance->queue.tail %= instance->queue.size;
      uint8_t index = instance->i2cLength
          - (((instance->queue.head + instance->queue.size) - instance->queue.tail) % instance->queue.size);
      if (bytes == index)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
    }
    else {
      I2C_SendData(I2Cx, instance->queue.buf[instance->queue.tail++]); //reg
      instance->queue.tail %= instance->queue.size;
      if (writing)
        instance->deviceIDSent = true; // 추가된 부분/ Write는 REP START가 없으므로
      if (reading || !bytes)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
    }
  }

  /* END */
  if (end != false) {
    instance->deviceIDSent = false;
    if (final_stop)
      I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    instance->busy = false;
  }
}

void I2C1_ER_IRQHandler(void) {
  i2c_er_handler(&i2cPort1);
}

void I2C1_EV_IRQHandler(void) {
  i2c_ev_handler(&i2cPort1);
}

void I2C2_ER_IRQHandler(void) {
  i2c_er_handler(&i2cPort2);
}

void I2C2_EV_IRQHandler(void) {
  i2c_ev_handler(&i2cPort2);
}
