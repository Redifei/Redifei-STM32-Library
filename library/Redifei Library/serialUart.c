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

static red_serialUartPort_t* serialPrintfPort;
static red_serialUartPort_t serialUartPorts[RED_SERIAL_UART_PORT_MAX];
static red_serialUart_setting_t serialUartSettings[RED_SERIAL_UART_PORT_MAX];
static red_serialUartPortBuf_t serialUartPortBuf[RED_SERIAL_UART_PORT_MAX];

static bool red_available(struct red_serialUartPort* this) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->setting));

  return this->setting->rxQueue.head != this->setting->rxQueue.tail;
}

static void red_putchar(struct red_serialUartPort* this, char c) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->setting));

  const red_serialUart_hardware_t* serialHW = this->setting->hw;
  red_serialUart_userSetting_t* serialUserSetting = this->setting->userSetting;

  if (serialUserSetting->serialMode == RED_SERIAL_INTERRPUT_MODE) {
    this->setting->txQueue.buf[this->setting->txQueue.head++] = c;
    this->setting->txQueue.head %= this->setting->txQueue.size;
    USART_ITConfig(serialHW->uartPort, USART_IT_TXE, ENABLE);
  }
  else if (serialUserSetting->serialMode == RED_SERIAL_POLLING_MODE) {
    USART_SendData(serialHW->uartPort, c);
    while (USART_GetFlagStatus(serialHW->uartPort, USART_FLAG_TXE) == RESET)
      ;
  }
}

static char red_getchar(struct red_serialUartPort* this) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->setting));

  char c = '\0';
  const red_serialUart_hardware_t* serialHW = this->setting->hw;
  red_serialUart_userSetting_t* serialUserSetting = this->setting->userSetting;

  if (serialUserSetting->serialMode == RED_SERIAL_INTERRPUT_MODE) {
    while (1) {
      if (this->setting->rxQueue.head != this->setting->rxQueue.tail) {
        c = this->setting->rxQueue.buf[this->setting->rxQueue.tail++];
        this->setting->rxQueue.tail %= this->setting->rxQueue.size;
      }
//    if (c != '\0') // wait when input data via serial
      break;
    }
  }
  else if (serialUserSetting->serialMode == RED_SERIAL_POLLING_MODE) {
    char c = '\0';
    while (USART_GetFlagStatus(serialHW->uartPort, USART_FLAG_RXNE) == RESET)
      ;
    c = (char) USART_ReceiveData(serialHW->uartPort);
  }
  return c;
}

static void red_putc(void *p, char c) {
  red_putchar(serialPrintfPort, c);
}

void red_Printf(struct red_serialUartPort* this, char *format, ...) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->setting));

  serialPrintfPort = this;
  va_list va;
  va_start(va, format);
  tfp_format(NULL, red_putc, format, va);
  va_end(va);
}

static void red_serialUartConfig(struct red_serialUartPort* this) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->setting));

  const red_serialUart_hardware_t* serialHW = this->setting->hw;
  red_serialUart_userSetting_t* serialUserSetting = this->setting->userSetting;

  if (serialUserSetting->serialMode == RED_SERIAL_INTERRPUT_MODE || serialUserSetting->serialMode == RED_SERIAL_POLLING_MODE) {
    RCC_APB2PeriphClockCmd(serialHW->rx_gpioClock, ENABLE);
    RCC_APB2PeriphClockCmd(serialHW->tx_gpioClock, ENABLE);

    if (serialHW->uartClock != RCC_APB2Periph_USART1)
      RCC_APB1PeriphClockCmd(serialHW->uartClock, ENABLE); // 2345
    else
      RCC_APB2PeriphClockCmd(serialHW->uartClock, ENABLE); // 1

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = serialHW->tx_gpioPin; // tx pa9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(serialHW->tx_gpioPort, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = serialHW->rx_gpioPin; // rx pa10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(serialHW->rx_gpioPort, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = serialUserSetting->baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = serialUserSetting->stopbit;
    USART_InitStructure.USART_Parity = serialUserSetting->parity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(serialHW->uartPort, &USART_InitStructure);

    if (serialUserSetting->serialMode == RED_SERIAL_INTERRPUT_MODE) {
      USART_ITConfig(serialHW->uartPort, USART_IT_RXNE, ENABLE);

      NVIC_InitTypeDef NVIC_InitStructure;
      NVIC_InitStructure.NVIC_IRQChannel = serialHW->uartIRQ;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
    }

    USART_Cmd(serialHW->uartPort, ENABLE);
  }
  else {
    // SOFT SERIAL
  }
}

red_serialUartPort_t* redSerialUartInit(uint8_t serialUartPortNum, red_serialUart_userSetting_t* userSetting) {
  assert_param(IS_VAILD_SERIAL_PORT_NUM(serialUartPortNum));

  red_serialUartPort_t* serialUartPort = &serialUartPorts[serialUartPortNum];

  serialUartSettings[serialUartPortNum].hw = &redSerialUartHardWareMap[serialUartPortNum];
  serialUartSettings[serialUartPortNum].userSetting = userSetting;
  serialUartPort->setting = &serialUartSettings[serialUartPortNum];

  serialUartPort->setting->txQueue.buf = serialUartPortBuf[serialUartPortNum].txBuf;
  serialUartPort->setting->txQueue.size = BUF_SIZE;
  serialUartPort->setting->rxQueue.buf = serialUartPortBuf[serialUartPortNum].rxBuf;
  serialUartPort->setting->rxQueue.size = BUF_SIZE;

  serialUartPort->putChar = red_putchar;
  serialUartPort->getChar = red_getchar;
  serialUartPort->printf = red_Printf;
  serialUartPort->available = red_available;

  red_serialUartConfig(serialUartPort);
  return serialUartPort;
}

static void red_serialUart_handler(struct red_serialUartPort* this) {
  assert_param(IS_CONFIGED_SERIAL_PORT(this->setting));

  const red_serialUart_hardware_t* serialHW = this->setting->hw;

  if (USART_GetITStatus(serialHW->uartPort, USART_IT_RXNE) != RESET) {
    this->setting->rxQueue.buf[this->setting->rxQueue.head++] = USART_ReceiveData(serialHW->uartPort);
    this->setting->rxQueue.head %= this->setting->rxQueue.size;
  }
  if (USART_GetITStatus(serialHW->uartPort, USART_IT_TXE) != RESET) {
    USART_SendData(serialHW->uartPort, this->setting->txQueue.buf[this->setting->txQueue.tail++]);
    this->setting->txQueue.tail %= this->setting->txQueue.size;
    if (this->setting->txQueue.head == this->setting->txQueue.tail)
      USART_ITConfig(serialHW->uartPort, USART_IT_TXE, DISABLE);
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
