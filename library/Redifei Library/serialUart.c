/*
 serialUart.c - serialUart library
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
#include "printf.h"
#include "serialUart.h"

#define BUF_SIZE 256

typedef struct {
  uint8_t txBuf[BUF_SIZE];
  uint8_t rxBuf[BUF_SIZE];
} red_serialUartPortBuf_t;

static red_serialUartDevice_t* serialPrintfPort;
static red_serialUartDevice_t serialUartPorts[RED_SERIAL_UART_PORT_MAX];
static red_serialUart_param_t serialUartSettings[RED_SERIAL_UART_PORT_MAX];
static red_serialUartPortBuf_t serialUartPortBuf[RED_SERIAL_UART_PORT_MAX];

static bool red_available(struct red_serialUartDevice* this) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->param));

  red_serialUart_param_t* param = this->param;
  return param->rxQueue.head != param->rxQueue.tail;
}

static void red_putchar(struct red_serialUartDevice* this, char c) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->param));

  red_serialUart_param_t* param = this->param;
  const red_serialUart_hardware_t* hw = param->hw;
  red_serialUart_userSetting_t* userSetting = param->userSetting;

  if (userSetting->serialMode == RED_SERIAL_INTERRPUT_MODE) {
    param->txQueue.buf[param->txQueue.head++] = c;
    param->txQueue.head %= param->txQueue.size;
    USART_ITConfig(hw->uartPort, USART_IT_TXE, ENABLE);
  }
  else if (userSetting->serialMode == RED_SERIAL_POLLING_MODE) {
    USART_SendData(hw->uartPort, c);
    while (USART_GetFlagStatus(hw->uartPort, USART_FLAG_TXE) == RESET)
      ;
  }
}

static char red_getchar(struct red_serialUartDevice* this) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->param));

  char c = '\0';
  red_serialUart_param_t* param = this->param;
  const red_serialUart_hardware_t* hw = param->hw;
  red_serialUart_userSetting_t* userSetting = param->userSetting;

  if (userSetting->serialMode == RED_SERIAL_INTERRPUT_MODE) {
    while (1) {
      if (param->rxQueue.head != param->rxQueue.tail) {
        c = param->rxQueue.buf[param->rxQueue.tail++];
        param->rxQueue.tail %= param->rxQueue.size;
      }
//    if (c != '\0') // wait when input data via serial
      break;
    }
  }
  else if (userSetting->serialMode == RED_SERIAL_POLLING_MODE) {
    char c = '\0';
    while (USART_GetFlagStatus(hw->uartPort, USART_FLAG_RXNE) == RESET)
      ;
    c = (char) USART_ReceiveData(hw->uartPort);
  }
  return c;
}

static void red_putc(void *p, char c) {
  red_putchar(serialPrintfPort, c);
}

void red_Printf(struct red_serialUartDevice* this, char *format, ...) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->param));

  serialPrintfPort = this;
  va_list va;
  va_start(va, format);
  tfp_format(NULL, red_putc, format, va);
  va_end(va);
}

static void red_serialUartConfig(struct red_serialUartDevice* this) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->param));

  red_serialUart_param_t* param = this->param;
  const red_serialUart_hardware_t* hw = param->hw;
  red_serialUart_userSetting_t* userSetting = param->userSetting;

  if (userSetting->serialMode == RED_SERIAL_INTERRPUT_MODE || userSetting->serialMode == RED_SERIAL_POLLING_MODE) {
    RCC_APB2PeriphClockCmd(hw->rx_gpioClock, ENABLE);
    RCC_APB2PeriphClockCmd(hw->tx_gpioClock, ENABLE);

    if (hw->uartClock != RCC_APB2Periph_USART1)
      RCC_APB1PeriphClockCmd(hw->uartClock, ENABLE); // 2345
    else
      RCC_APB2PeriphClockCmd(hw->uartClock, ENABLE); // 1

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = hw->tx_gpioPin; // tx pa9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(hw->tx_gpioPort, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = hw->rx_gpioPin; // rx pa10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(hw->rx_gpioPort, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = userSetting->baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = userSetting->stopbit;
    USART_InitStructure.USART_Parity = userSetting->parity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(hw->uartPort, &USART_InitStructure);

    if (userSetting->serialMode == RED_SERIAL_INTERRPUT_MODE) {
      USART_ITConfig(hw->uartPort, USART_IT_RXNE, ENABLE);

      NVIC_InitTypeDef NVIC_InitStructure;
      NVIC_InitStructure.NVIC_IRQChannel = hw->uartIRQ;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
    }

    USART_Cmd(hw->uartPort, ENABLE);
  }
  else {
    // SOFT SERIAL
  }
}

red_serialUartDevice_t* redSerialUartInit(uint8_t serialUartPortNum, red_serialUart_userSetting_t* userSetting) {
  assert_param(IS_VAILD_SERIAL_PORT_NUM(serialUartPortNum));

  red_serialUartDevice_t* serialUartPort = &serialUartPorts[serialUartPortNum];

  serialUartSettings[serialUartPortNum].hw = &redSerialUartHardWareMap[serialUartPortNum];
  serialUartSettings[serialUartPortNum].userSetting = userSetting;
  serialUartPort->param = &serialUartSettings[serialUartPortNum];

  serialUartPort->param->txQueue.buf = serialUartPortBuf[serialUartPortNum].txBuf;
  serialUartPort->param->txQueue.size = BUF_SIZE;
  serialUartPort->param->rxQueue.buf = serialUartPortBuf[serialUartPortNum].rxBuf;
  serialUartPort->param->rxQueue.size = BUF_SIZE;

  serialUartPort->putChar = red_putchar;
  serialUartPort->getChar = red_getchar;
  serialUartPort->printf = red_Printf;
  serialUartPort->available = red_available;

  red_serialUartConfig(serialUartPort);
  return serialUartPort;
}

static void red_serialUart_handler(struct red_serialUartDevice* this) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->param));

  red_serialUart_param_t* param = this->param;
  const red_serialUart_hardware_t* hw = param->hw;

  if (USART_GetITStatus(hw->uartPort, USART_IT_RXNE) != RESET) {
    param->rxQueue.buf[param->rxQueue.head++] = USART_ReceiveData(hw->uartPort);
    param->rxQueue.head %= param->rxQueue.size;
  }
  if (USART_GetITStatus(hw->uartPort, USART_IT_TXE) != RESET) {
    USART_SendData(hw->uartPort, param->txQueue.buf[param->txQueue.tail++]);
    param->txQueue.tail %= param->txQueue.size;
    if (param->txQueue.head == param->txQueue.tail)
      USART_ITConfig(hw->uartPort, USART_IT_TXE, DISABLE);
  }
}

void USART1_IRQHandler() {
  red_serialUart_handler(&serialUartPorts[RED_SERIAL_UART_PORT_1]);
}

void USART2_IRQHandler() {
  red_serialUart_handler(&serialUartPorts[RED_SERIAL_UART_PORT_2]);
}

void USART3_IRQHandler() {
  red_serialUart_handler(&serialUartPorts[RED_SERIAL_UART_PORT_3]);
}
