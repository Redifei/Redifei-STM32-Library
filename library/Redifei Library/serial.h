/*
 * Redifei: Serial Port Library
 * Based on BaseFlight Project
 *
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 *
 *
 * TODO(Serial):
 * Support softWare mode
 * FIXME(Serial):
 * Done : Can't read null
 *
 *
 * -Functions
 * openSerialx : x(1;3)
 * ->putChar
 * ->getChar
 * ->printf
 * ->available
 */

#pragma once

#include <stdbool.h>
#include "queue.h"

typedef enum {
  SERIAL_INTERRPUT_MODE, SERIAL_POLLING_MODE, SERIAL_SOFTWARE_MODE,
} serialMode_t;

typedef struct {
  /* 하드웨어 기본 설정*/
  uint16_t tx_gpioPin;
  GPIO_TypeDef* tx_gpioPort;
  uint32_t tx_gpioClock;

  uint16_t rx_gpioPin;
  GPIO_TypeDef* rx_gpioPort;
  uint32_t rx_gpioClock;

  /* 유아트 포트 기본 설정*/
  USART_TypeDef* uartPort;
  uint32_t uartClock;

  IRQn_Type uartIRQ;

  /* 유아트 포트 추가 설정 */
  uint32_t uartBaudrate;
  uint16_t uartParity;
  uint16_t uartStopbit;

  /* 시리얼 통신에서 사용할 큐 */
  Qtype_t txQueue;
  Qtype_t rxQueue;

  /* 시리얼 포트에서 사용할 함수 */
  void (*putChar)(char);
  char (*getChar)();
  void (*printf)(char*, ...);
  bool (*available)();

  serialMode_t serialMode;
} serialUartPort_t;

serialUartPort_t* openSerial1(serialMode_t mode);
serialUartPort_t* openSerial2(serialMode_t mode);
serialUartPort_t* openSerial3(serialMode_t mode);
