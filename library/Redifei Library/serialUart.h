/*
 serialUart.h - serialUart library
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

#include <stdbool.h>
#include "queue.h"

enum {
  RED_SERIAL_UART_PORT_1,
  RED_SERIAL_UART_PORT_2,
  RED_SERIAL_UART_PORT_3,
  RED_SERIAL_UART_PORT_MAX,
};

// Enum
typedef enum {
  RED_SERIAL_INTERRPUT_MODE, RED_SERIAL_POLLING_MODE, RED_SERIAL_SOFTWARE_MODE,
} red_serialMode_t;

// HardWare List
typedef struct {
  GPIO_TypeDef* tx_gpioPort;
  uint32_t tx_gpioClock;
  uint16_t tx_gpioPin;

  GPIO_TypeDef* rx_gpioPort;
  uint32_t rx_gpioClock;
  uint16_t rx_gpioPin;

  USART_TypeDef* uartPort;
  uint32_t uartClock;

  IRQn_Type uartIRQ;
} red_serialUart_hardware_t;

static const red_serialUart_hardware_t redSerialUartHardWareMap[RED_SERIAL_UART_PORT_MAX] = {
    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_9, GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_10, USART1, RCC_APB2Periph_USART1, USART1_IRQn },
    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_2, GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_3, USART2, RCC_APB1Periph_USART2, USART2_IRQn },
    { GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_10, GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_11, USART3, RCC_APB1Periph_USART3, USART3_IRQn },
};

// User Setting List
typedef struct {
  red_serialMode_t serialMode;
  uint32_t baudrate;
  uint16_t parity;
  uint16_t stopbit;
//  uint16_t bufSize;
} red_serialUart_userSetting_t;

// Serial Setting List
typedef struct {
  const red_serialUart_hardware_t* hw;
  red_serialUart_userSetting_t* userSetting;
  Qtype_t txQueue;
  Qtype_t rxQueue;
} red_serialUart_setting_t;

// Serial Port List
typedef struct red_serialUartPort {
  red_serialUart_setting_t* setting;
  void (*putChar)(struct red_serialUartPort* this, char);
  char (*getChar)(struct red_serialUartPort* this);
  void (*printf)(struct red_serialUartPort* this, char *format, ...);
  bool (*available)(struct red_serialUartPort* this);
} red_serialUartPort_t;

#define IS_CONFIGED_SERIAL_PORT(THIS_SETTING) (THIS_SETTING != NULL)
#define IS_VAILD_SERIAL_PORT_NUM(SERIAL_NUM) (SERIAL_NUM < RED_SERIAL_UART_PORT_MAX)

red_serialUartPort_t* redSerialUartInit(uint8_t serialUartPortNum, red_serialUart_userSetting_t* userSetting);
