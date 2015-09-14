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

// Based on I2C optimized examples(an2824) for Stmicroelectronics
// TODO: Add Generate Stop Select

#include <stdlib.h>
#include <stdbool.h>
#include <stm32f10x_conf.h>
#include "systickTimer.h"
#include "i2c.h"

#define BUF_SIZE 32

/* I2C SPE mask */
#define CR1_PE_Set              ((uint16_t)0x0001)
#define CR1_PE_Reset            ((uint16_t)0xFFFE)

/* I2C START mask */
#define CR1_START_Set           ((uint16_t)0x0100)
#define CR1_START_Reset         ((uint16_t)0xFEFF)

#define CR1_POS_Set           ((uint16_t)0x0800)
#define CR1_POS_Reset         ((uint16_t)0xF7FF)

/* I2C STOP mask */
#define CR1_STOP_Set            ((uint16_t)0x0200)
#define CR1_STOP_Reset          ((uint16_t)0xFDFF)

/* I2C ACK mask */
#define CR1_ACK_Set             ((uint16_t)0x0400)
#define CR1_ACK_Reset           ((uint16_t)0xFBFF)

/* I2C ENARP mask */
#define CR1_ENARP_Set           ((uint16_t)0x0010)
#define CR1_ENARP_Reset         ((uint16_t)0xFFEF)

/* I2C NOSTRETCH mask */
#define CR1_NOSTRETCH_Set       ((uint16_t)0x0080)
#define CR1_NOSTRETCH_Reset     ((uint16_t)0xFF7F)

/* I2C registers Masks */
#define CR1_CLEAR_Mask          ((uint16_t)0xFBF5)

/* I2C DMAEN mask */
#define CR2_DMAEN_Set           ((uint16_t)0x0800)
#define CR2_DMAEN_Reset         ((uint16_t)0xF7FF)

/* I2C LAST mask */
#define CR2_LAST_Set            ((uint16_t)0x1000)
#define CR2_LAST_Reset          ((uint16_t)0xEFFF)

/* I2C FREQ mask */
#define CR2_FREQ_Reset          ((uint16_t)0xFFC0)

/* I2C ADD0 mask */
#define OAR1_ADD0_Set           ((uint16_t)0x0001)
#define OAR1_ADD0_Reset         ((uint16_t)0xFFFE)

/* I2C ENDUAL mask */
#define OAR2_ENDUAL_Set         ((uint16_t)0x0001)
#define OAR2_ENDUAL_Reset       ((uint16_t)0xFFFE)

/* I2C ADD2 mask */
#define OAR2_ADD2_Reset         ((uint16_t)0xFF01)

/* I2C F/S mask */
#define CCR_FS_Set              ((uint16_t)0x8000)

/* I2C CCR mask */
#define CCR_CCR_Set             ((uint16_t)0x0FFF)

/* I2C FLAG mask */
#define FLAG_Mask               ((uint32_t)0x00FFFFFF)

/* I2C Interrupt Enable mask */
#define ITEN_Mask               ((uint32_t)0x07000000)

typedef enum {
  I2C_DIRECTION_TX = 0,
  I2C_DIRECTION_RX,
} red_i2cDirection;

// use input pointer buf as original buf
//typedef struct {
//  uint8_t buf[BUF_SIZE];
//} red_i2cPortBuf_t;

static red_i2cDevice_t i2cPorts[RED_I2C_PORT_MAX];
static red_i2c_param_t i2cSettings[RED_I2C_PORT_MAX];
//static red_i2cPortBuf_t i2cPortBuf[RED_I2C_PORT_MAX];

static bool red_i2cReadBytes(struct red_i2cDevice* this, uint8_t SlaveAddress, uint8_t NumByteToRead, uint8_t* pBuffer) {
  assert_param(IS_CONFIGED_I2C_PORT(this->param));

  red_i2c_param_t* param = this->param;
  const red_i2c_hardware_t* hw = param->hw;
  red_i2c_userSetting_t* userSetting = param->userSetting;

  I2C_TypeDef* I2Cx = hw->i2cPort;
  red_i2cMode_t Mode = userSetting->i2cMode;

  volatile uint32_t temp = 0;
  volatile uint32_t Timeout = 0;

  /* Enable I2C errors interrupts (used in all modes: Polling, DMA and Interrupts */
  I2Cx->CR2 |= I2C_IT_ERR;

  if (Mode == RED_I2C_POLLING_MODE) /* I2Cx Master Reception using Polling */
  {

    if (NumByteToRead == 1) {
      Timeout = userSetting->timeOut;
      /* Send START condition */
      I2Cx->CR1 |= CR1_START_Set;
      /* Wait until SB flag is set: EV5  */
      while ((I2Cx->SR1 & 0x0001) != 0x0001) {
        if (Timeout-- == 0)
          return true;
      }
      /* Send slave address */
      /* Reset the address bit0 for read */
      SlaveAddress = (SlaveAddress << 1) | OAR1_ADD0_Set;
      param->address = SlaveAddress;
      /* Send the slave address */
      I2Cx->DR = param->address;
      /* Wait until ADDR is set: EV6_3, then program ACK = 0, clear ADDR
       and program the STOP just after ADDR is cleared. The EV6_3
       software sequence must complete before the current byte end of transfer.*/
      /* Wait until ADDR is set */
      Timeout = userSetting->timeOut;
      while ((I2Cx->SR1 & 0x0002) != 0x0002) {
        if (Timeout-- == 0)
          return true;
      }
      /* Clear ACK bit */
      I2Cx->CR1 &= CR1_ACK_Reset;
      /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
       software sequence must complete before the current byte end of transfer */
      __disable_irq();
      /* Clear ADDR flag */
      temp = I2Cx->SR2;
      /* Program the STOP */
      I2Cx->CR1 |= CR1_STOP_Set;
      /* Re-enable IRQs */
      __enable_irq();
      Timeout = userSetting->timeOut;
      /* Wait until a data is received in DR register (RXNE = 1) EV7 */
      while ((I2Cx->SR1 & 0x00040) != 0x000040) {
        if (Timeout-- == 0)
          return true;
      }
      /* Read the data */
      *pBuffer = I2Cx->DR;
      /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
      while ((I2Cx->CR1 & 0x200) == 0x200) {
        if (Timeout-- == 0)
          return true;
      }
      /* Enable Acknowledgement to be ready for another reception */
      I2Cx->CR1 |= CR1_ACK_Set;

    }

    else if (NumByteToRead == 2) {
      /* Set POS bit */
      I2Cx->CR1 |= CR1_POS_Set;
      Timeout = userSetting->timeOut;
      /* Send START condition */
      I2Cx->CR1 |= CR1_START_Set;
      /* Wait until SB flag is set: EV5 */
      while ((I2Cx->SR1 & 0x0001) != 0x0001) {
        if (Timeout-- == 0)
          return true;
      }
      Timeout = userSetting->timeOut;
      /* Send slave address */
      /* Set the address bit0 for read */
      SlaveAddress = (SlaveAddress << 1) | OAR1_ADD0_Set;
      param->address = SlaveAddress;
      /* Send the slave address */
      I2Cx->DR = param->address;
      /* Wait until ADDR is set: EV6 */
      while ((I2Cx->SR1 & 0x0002) != 0x0002) {
        if (Timeout-- == 0)
          return true;
      }
      /* EV6_1: The acknowledge disable should be done just after EV6,
       that is after ADDR is cleared, so disable all active IRQs around ADDR clearing and
       ACK clearing */
      __disable_irq();
      /* Clear ADDR by reading SR2 register  */
      temp = I2Cx->SR2;
      /* Clear ACK */
      I2Cx->CR1 &= CR1_ACK_Reset;
      /*Re-enable IRQs */
      __enable_irq();
      Timeout = userSetting->timeOut;
      /* Wait until BTF is set */
      while ((I2Cx->SR1 & 0x00004) != 0x000004) {
        if (Timeout-- == 0)
          return true;
      }
      /* Disable IRQs around STOP programming and data reading because of the limitation ?*/
      __disable_irq();
      /* Program the STOP */
      I2C_GenerateSTOP(I2Cx, ENABLE);
      /* Read first data */
      *pBuffer = I2Cx->DR;
      /* Re-enable IRQs */
      __enable_irq();
      /**/
      pBuffer++;
      /* Read second data */
      *pBuffer = I2Cx->DR;
      Timeout = userSetting->timeOut;
      /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
      while ((I2Cx->CR1 & 0x200) == 0x200) {
        if (Timeout-- == 0)
          return true;
      }
      /* Enable Acknowledgement to be ready for another reception */
      I2Cx->CR1 |= CR1_ACK_Set;
      /* Clear POS bit */
      I2Cx->CR1 &= CR1_POS_Reset;

    }

    else

    {

      Timeout = userSetting->timeOut;
      /* Send START condition */
      I2Cx->CR1 |= CR1_START_Set;
      /* Wait until SB flag is set: EV5 */
      while ((I2Cx->SR1 & 0x0001) != 0x0001) {
        if (Timeout-- == 0)
          return true;
      }
      Timeout = userSetting->timeOut;
      /* Send slave address */
      /* Reset the address bit0 for write */
      SlaveAddress = (SlaveAddress << 1) | OAR1_ADD0_Set;
      param->address = SlaveAddress;
      /* Send the slave address */
      I2Cx->DR = param->address;
      /* Wait until ADDR is set: EV6 */
      while ((I2Cx->SR1 & 0x0002) != 0x0002) {
        if (Timeout-- == 0)
          return true;
      }
      /* Clear ADDR by reading SR2 status register */
      temp = I2Cx->SR2;
      /* While there is data to be read */
      while (NumByteToRead) {
        /* Receive bytes from first byte until byte N-3 */
        if (NumByteToRead != 3) {
          Timeout = userSetting->timeOut;
          /* Poll on BTF to receive data because in polling mode we can not guarantee the
           EV7 software sequence is managed before the current byte transfer completes */
          while ((I2Cx->SR1 & 0x00004) != 0x000004) {
            if (Timeout-- == 0)
              return true;
          }
          /* Read data */
          *pBuffer = I2Cx->DR;
          /* */
          pBuffer++;
          /* Decrement the read bytes counter */
          NumByteToRead--;
        }

        /* it remains to read three data: data N-2, data N-1, Data N */
        if (NumByteToRead == 3) {

          Timeout = userSetting->timeOut;
          /* Wait until BTF is set: Data N-2 in DR and data N -1 in shift register */
          while ((I2Cx->SR1 & 0x00004) != 0x000004) {
            if (Timeout-- == 0)
              return true;
          }
          /* Clear ACK */
          I2Cx->CR1 &= CR1_ACK_Reset;

          /* Disable IRQs around data reading and STOP programming because of the
           limitation ? */
          __disable_irq();
          /* Read Data N-2 */
          *pBuffer = I2Cx->DR;
          /* Increment */
          pBuffer++;
          /* Program the STOP */
          I2Cx->CR1 |= CR1_STOP_Set;
          /* Read DataN-1 */
          *pBuffer = I2Cx->DR;
          /* Re-enable IRQs */
          __enable_irq();
          /* Increment */
          pBuffer++;
          Timeout = userSetting->timeOut;
          /* Wait until RXNE is set (DR contains the last data) */
          while ((I2Cx->SR1 & 0x00040) != 0x000040) {
            if (Timeout-- == 0)
              return true;
          }
          /* Read DataN */
          *pBuffer = I2Cx->DR;
          /* Reset the number of bytes to be read by master */
          NumByteToRead = 0;

        }
      }
      Timeout = userSetting->timeOut;
      /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
      while ((I2Cx->CR1 & 0x200) == 0x200) {
        if (Timeout-- == 0)
          return true;
      }
      /* Enable Acknowledgement to be ready for another reception */
      I2Cx->CR1 |= CR1_ACK_Set;

    }

  }

  else /* I2Cx Master Reception using Interrupts with highest priority in an application */
  {
    this->param->buffer = pBuffer;
    /* Enable EVT IT*/
    I2Cx->CR2 |= I2C_IT_EVT;
    /* Enable BUF IT */
    I2Cx->CR2 |= I2C_IT_BUF;
    /* Set the I2C direction to reception */
    param->direction = I2C_DIRECTION_RX;
    SlaveAddress = (SlaveAddress << 1) | OAR1_ADD0_Set;
    param->address = SlaveAddress;
    param->numbOfBytes = NumByteToRead;
    /* Send START condition */
    I2Cx->CR1 |= CR1_START_Set;
    Timeout = userSetting->timeOut;
    /* Wait until the START condition is generated on the bus: START bit is cleared by hardware */
    while ((I2Cx->CR1 & 0x100) == 0x100) {
      if (Timeout-- == 0)
        return true;
    }
    Timeout = userSetting->timeOut;
    /* Wait until BUSY flag is reset (until a STOP is generated) */
    while ((I2Cx->SR2 & 0x0002) == 0x0002) {
      if (Timeout-- == 0)
        return true;
    }
    /* Enable Acknowledgement to be ready for another reception */
    I2Cx->CR1 |= CR1_ACK_Set;
  }

  return false;
}
bool red_i2cRead1Byte(struct red_i2cDevice* this, uint8_t SlaveAddress, uint8_t* pBuffer) {
  assert_param(IS_CONFIGED_I2C_PORT(this->param));
  return red_i2cReadBytes(this, SlaveAddress, 1, pBuffer);
}

static bool red_i2cWriteBytes(struct red_i2cDevice* this, uint8_t SlaveAddress, uint8_t NumByteToWrite, uint8_t* pBuffer) {
  assert_param(IS_CONFIGED_I2C_PORT(this->param));

  red_i2c_param_t* param = this->param;
  const red_i2c_hardware_t* hw = param->hw;
  red_i2c_userSetting_t* userSetting = param->userSetting;

  I2C_TypeDef* I2Cx = hw->i2cPort;
  red_i2cMode_t Mode = userSetting->i2cMode;

  volatile uint32_t temp = 0;
  volatile uint32_t Timeout = 0;

  /* Enable true IT (used in all modes: DMA, Polling and Interrupts */
  I2Cx->CR2 |= I2C_IT_ERR;
  if (Mode == RED_I2C_POLLING_MODE) /* I2Cx Master Transmission using Polling */
  {

    Timeout = userSetting->timeOut;
    /* Send START condition */
    I2Cx->CR1 |= CR1_START_Set;
    /* Wait until SB flag is set: EV5 */
    while ((I2Cx->SR1 & 0x0001) != 0x0001) {
      if (Timeout-- == 0)
        return true;
    }

    /* Send slave address */
    /* Reset the address bit0 for write*/
    SlaveAddress = (SlaveAddress << 1) & OAR1_ADD0_Reset;
    this->param->address = SlaveAddress;
    /* Send the slave address */
    I2Cx->DR = this->param->address;
    Timeout = userSetting->timeOut;
    /* Wait until ADDR is set: EV6 */
    while ((I2Cx->SR1 & 0x0002) != 0x0002) {
      if (Timeout-- == 0)
        return true;
    }

    /* Clear ADDR flag by reading SR2 register */
    temp = I2Cx->SR2;
    /* Write the first data in DR register (EV8_1) */
    I2Cx->DR = *pBuffer;
    /* Increment */
    pBuffer++;
    /* Decrement the number of bytes to be written */
    NumByteToWrite--;
    Timeout = userSetting->timeOut;
    /* While there is data to be written */
    while (NumByteToWrite--) {
      /* Poll on BTF to receive data because in polling mode we can not guarantee the
       EV8 software sequence is managed before the current byte transfer completes */
      while ((I2Cx->SR1 & 0x00004) != 0x000004) {
        if (Timeout-- == 0)
          return true;
      }
      /* Send the current byte */
      I2Cx->DR = *pBuffer;
      /* Point to the next byte to be written */
      pBuffer++;
    }
    Timeout = userSetting->timeOut;
    /* EV8_2: Wait until BTF is set before programming the STOP */
    while ((I2Cx->SR1 & 0x00004) != 0x000004) {
      if (Timeout-- == 0)
        return true;
    }
    /* Send STOP condition */
    I2Cx->CR1 |= CR1_STOP_Set;
    Timeout = userSetting->timeOut;
    /* Make sure that the STOP bit is cleared by Hardware */
    while ((I2Cx->CR1 & 0x200) == 0x200) {
      if (Timeout-- == 0)
        return true;
    }

  }

  else /* I2Cx Master Transmission using Interrupt with highest priority in the application */

  {
    this->param->buffer = pBuffer;
    /* Enable EVT IT*/
    I2Cx->CR2 |= I2C_IT_EVT;
    /* Enable BUF IT */
    I2Cx->CR2 |= I2C_IT_BUF;
    /* Set the I2C direction to Transmission */
    this->param->direction = I2C_DIRECTION_TX;
    SlaveAddress = (SlaveAddress << 1) & OAR1_ADD0_Reset;
    this->param->address = SlaveAddress;
    this->param->numbOfBytes = NumByteToWrite;
    /* Send START condition */
    I2Cx->CR1 |= CR1_START_Set;
    Timeout = userSetting->timeOut;
    /* Wait until the START condition is generated on the bus: the START bit is cleared by hardware */
    while ((I2Cx->CR1 & 0x100) == 0x100) {
      if (Timeout-- == 0)
        return true;
    }
    Timeout = userSetting->timeOut;
    /* Wait until BUSY flag is reset: a STOP has been generated on the bus signaling the end
     of transmission */
    while ((I2Cx->SR2 & 0x0002) == 0x0002) {
      if (Timeout-- == 0)
        return true;
    }
  }

  return false;
}

bool red_i2cWrite1Byte(struct red_i2cDevice* this, uint8_t SlaveAddress, uint8_t buffer) {
  assert_param(IS_CONFIGED_I2C_PORT(this->param));
  return red_i2cWriteBytes(this, SlaveAddress, 1, &buffer);
}

static bool i2cReset(struct red_i2cDevice* this) {
  const red_i2c_hardware_t* hw = this->param->hw;
  red_i2c_userSetting_t* userSetting = this->param->userSetting;

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = hw->scl_gpioPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(hw->scl_gpioPort, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = hw->sda_gpioPin;
  GPIO_Init(hw->sda_gpioPort, &GPIO_InitStructure);

  GPIO_WriteBit(hw->scl_gpioPort, hw->scl_gpioPin, Bit_SET);
  GPIO_WriteBit(hw->sda_gpioPort, hw->sda_gpioPin, Bit_SET);

  uint8_t i;
  for (i = 0; i < 8; i++) {
    // Wait for any clock stretching to finish
    volatile uint32_t Timeout = 0xffff;
    while (!GPIO_ReadInputDataBit(hw->scl_gpioPort, hw->scl_gpioPin)) {
      if (Timeout-- == 0)
        return true;
    }
    delayMicroseconds(10);

    // Pull low
    GPIO_WriteBit(hw->scl_gpioPort, hw->scl_gpioPin, Bit_RESET); // Set bus low
    delayMicroseconds(10);
    // Release high again
    GPIO_WriteBit(hw->scl_gpioPort, hw->scl_gpioPin, Bit_SET); // Set bus high
    delayMicroseconds(10);
  }

  GPIO_WriteBit(hw->sda_gpioPort, hw->sda_gpioPin, Bit_RESET); // Set bus data low
  delayMicroseconds(10);
  GPIO_WriteBit(hw->scl_gpioPort, hw->scl_gpioPin, Bit_RESET); // Set bus scl low
  delayMicroseconds(10);
  GPIO_WriteBit(hw->scl_gpioPort, hw->scl_gpioPin, Bit_SET); // Set bus scl high
  delayMicroseconds(10);
  GPIO_WriteBit(hw->sda_gpioPort, hw->sda_gpioPin, Bit_SET); // Set bus sda high
  return false;
}

static void red_i2cConfig(struct red_i2cDevice* this) {
  assert_param(IS_CONFIGED_I2C_PORT(this->param));

  red_i2c_param_t* param = this->param;
  const red_i2c_hardware_t* hw = param->hw;
  red_i2c_userSetting_t* userSetting = param->userSetting;

  I2C_TypeDef* I2Cx = hw->i2cPort;

  /* GPIOB clock enable */
  RCC_APB2PeriphClockCmd(hw->scl_gpioClock, ENABLE);
  RCC_APB2PeriphClockCmd(hw->sda_gpioClock, ENABLE);

  /* I2C1 clock enable */
  RCC_APB1PeriphClockCmd(hw->i2cClock, ENABLE);

  i2cReset(this);

  /* I2C1 SDA and SCL configuration */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = hw->scl_gpioPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(hw->scl_gpioPort, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = hw->sda_gpioPin;
  GPIO_Init(hw->sda_gpioPort, &GPIO_InitStructure);

  /* Enable I2C1 reset state */
  RCC_APB1PeriphResetCmd(hw->i2cClock, ENABLE);
  /* Release I2C1 from reset state */
  RCC_APB1PeriphResetCmd(hw->i2cClock, DISABLE);

  /* I2C1 and I2C2 configuration */
  I2C_InitTypeDef I2C_InitStructure;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = userSetting->clockSpeed;
  I2C_Init(hw->i2cPort, &I2C_InitStructure);

  // I2C ER Interrupt
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = hw->i2cErIRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // I2C EV Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = hw->i2cEvIRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);

  I2C_Cmd(hw->i2cPort, ENABLE);
}

red_i2cDevice_t* redI2cInit(uint8_t i2cPortNum, red_i2c_userSetting_t* userSetting) {
  assert_param(IS_VAILD_I2C_PORT_NUM(i2cPortNum));

  red_i2cDevice_t* i2cPort = &i2cPorts[i2cPortNum];

  i2cSettings[i2cPortNum].hw = &redI2cHardWareMap[i2cPortNum];
  i2cSettings[i2cPortNum].userSetting = userSetting;
  i2cPort->param = &i2cSettings[i2cPortNum];

//  i2cPort->param->buffer = i2cPortBuf[i2cPortNum].buf;

  i2cPort->reset = i2cReset;
  i2cPort->read1Byte = red_i2cRead1Byte;
  i2cPort->readBytes = red_i2cReadBytes;
  i2cPort->write1Byte = red_i2cWrite1Byte;
  i2cPort->writeBytes = red_i2cWriteBytes;

  red_i2cConfig(i2cPort);
  return i2cPort;
}

static void red_i2cER_handler(struct red_i2cDevice* this) {
  assert_param(IS_CONFIGED_I2C_PORT(this->param));

  red_i2c_param_t* param = this->param;
  const red_i2c_hardware_t* hw = param->hw;

  I2C_TypeDef* I2Cx = hw->i2cPort;

  volatile uint32_t SR1Register = 0;

  /* Read the I2Cx status register */
  SR1Register = I2Cx->SR1;
  /* If AF = 1 */
  if ((SR1Register & 0x0400) == 0x0400) {
    I2Cx->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  /* If ARLO = 1 */
  if ((SR1Register & 0x0200) == 0x0200) {
    I2Cx->SR1 &= 0xFBFF;
    SR1Register = 0;
  }
  /* If BERR = 1 */
  if ((SR1Register & 0x0100) == 0x0100) {
    I2Cx->SR1 &= 0xFEFF;
    SR1Register = 0;
  }

  /* If OVR = 1 */

  if ((SR1Register & 0x0800) == 0x0800) {
    I2Cx->SR1 &= 0xF7FF;
    SR1Register = 0;
  }
}

static void red_i2cEV_handler(struct red_i2cDevice* this) {
  assert_param(IS_CONFIGED_I2C_PORT(this->param));

  red_i2c_param_t* param = this->param;
  const red_i2c_hardware_t* hw = param->hw;
  red_i2c_userSetting_t* userSetting = param->userSetting;

  I2C_TypeDef* I2Cx = hw->i2cPort;

  volatile uint32_t SR1Register = 0;
  volatile uint32_t SR2Register = 0;

  /* Read the I2Cx SR1 and SR2 status registers */
  SR1Register = I2Cx->SR1;
  SR2Register = I2Cx->SR2;

  /* If I2Cx is slave (MSL flag = 0) */
  if ((SR2Register & 0x0001) != 0x0001) {
    /* If ADDR = 1: EV1 */
    if ((SR1Register & 0x0002) == 0x0002) {
      /* Clear SR1Register and SR2Register variables to prepare for next IT */
      SR1Register = 0;
      SR2Register = 0;
      /* Initialize the transmit/receive counters for next transmission/reception
       using Interrupt  */
      param->index = 0;
    }
    /* If TXE = 1: EV3 */
    if ((SR1Register & 0x0080) == 0x0080) {
      /* Write data in data register */
      I2Cx->DR = param->buffer[param->index++];
      SR1Register = 0;
      SR2Register = 0;
    }
    /* If RXNE = 1: EV2 */
    if ((SR1Register & 0x0040) == 0x0040) {
      /* Read data from data register */
      param->buffer[param->index++] = I2Cx->DR;
      SR1Register = 0;
      SR2Register = 0;

    }
    /* If STOPF =1: EV4 (Slave has detected a STOP condition on the bus */
    if ((SR1Register & 0x0010) == 0x0010) {
      I2Cx->CR1 |= CR1_PE_Set;
      SR1Register = 0;
      SR2Register = 0;

    }
  } /* End slave mode */

  /* If SB = 1, I2Cx master sent a START on the bus: EV5) */
  if ((SR1Register & 0x0001) == 0x0001) {

    /* Send the slave address for transmssion or for reception (according to the configured value
     in the write master write routine */
    I2Cx->DR = param->address;
    SR1Register = 0;
    SR2Register = 0;
  }
  /* If I2Cx is Master (MSL flag = 1) */

  if ((SR2Register & 0x0001) == 0x0001) {
    /* If ADDR = 1, EV6 */
    if ((SR1Register & 0x0002) == 0x0002) {
      /* Write the first data in case the Master is Transmitter */
      if (param->direction == I2C_DIRECTION_TX) {
        /* Initialize the Transmit counter */
        param->index = 0;
        /* Write the first data in the data register */
        I2Cx->DR = param->buffer[param->index++];
        /* Decrement the number of bytes to be written */
        param->numbOfBytes--;
        /* If no further data to be sent, disable the I2C BUF IT
         in order to not have a TxE  interrupt */
        if (param->numbOfBytes == 0) {
          I2Cx->CR2 &= (uint16_t) ~I2C_IT_BUF;
        }

      }
      /* Master Receiver */
      else

      {
        /* Initialize Receive counter */
        param->index = 0;
        /* At this stage, ADDR is cleared because both SR1 and SR2 were read.*/
        /* EV6_1: used for single byte reception. The ACK disable and the STOP
         Programming should be done just after ADDR is cleared. */
        if (param->numbOfBytes == 1) {
          /* Clear ACK */
          I2Cx->CR1 &= CR1_ACK_Reset;
          /* Program the STOP */
          I2Cx->CR1 |= CR1_STOP_Set;
        }
      }
      SR1Register = 0;
      SR2Register = 0;

    }
    /* Master transmits the remaing data: from data2 until the last one.  */
    /* If TXE is set */
    if ((SR1Register & 0x0084) == 0x0080) {
      /* If there is still data to write */
      if (param->numbOfBytes != 0) {
        /* Write the data in DR register */
        I2Cx->DR = param->buffer[param->index++];
        /* Decrment the number of data to be written */
        param->numbOfBytes--;
        /* If  no data remains to write, disable the BUF IT in order
         to not have again a TxE interrupt. */
        if (param->numbOfBytes == 0) {
          /* Disable the BUF IT */
          I2Cx->CR2 &= (uint16_t) ~I2C_IT_BUF;
        }
      }
      SR1Register = 0;
      SR2Register = 0;
    }
    /* If BTF and TXE are set (EV8_2), program the STOP */
    if ((SR1Register & 0x0084) == 0x0084) {

      /* Program the STOP */
      I2Cx->CR1 |= CR1_STOP_Set;
      /* Disable EVT IT In order to not have again a BTF IT */
      I2Cx->CR2 &= (uint16_t) ~I2C_IT_EVT;
      SR1Register = 0;
      SR2Register = 0;
    }
    /* If RXNE is set */
    if ((SR1Register & 0x0040) == 0x0040) {
      /* Read the data register */
      param->buffer[param->index++] = I2Cx->DR;
      /* Decrement the number of bytes to be read */
      param->numbOfBytes--;
      /* If it remains only one byte to read, disable ACK and program the STOP (EV7_1) */
      if (param->numbOfBytes == 1) {
        /* Clear ACK */
        I2Cx->CR1 &= CR1_ACK_Reset;
        /* Program the STOP */
        I2Cx->CR1 |= CR1_STOP_Set;
      }
      SR1Register = 0;
      SR2Register = 0;
    }

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
