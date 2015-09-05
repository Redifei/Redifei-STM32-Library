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

#include <stdlib.h>
#include <stdbool.h>
#include <stm32f10x_conf.h>
#include "printf.h"
#include "serial.h"

#define BUF_SIZE 256

static serialUartPort_t serialUartPort1;
static serialUartPort_t serialUartPort2;
static serialUartPort_t serialUartPort3;

static uint8_t serialUartPort1TxBuf[BUF_SIZE];
static uint8_t serialUartPort1RxBuf[BUF_SIZE];
static uint8_t serialUartPort2TxBuf[BUF_SIZE];
static uint8_t serialUartPort2RxBuf[BUF_SIZE];
static uint8_t serialUartPort3TxBuf[BUF_SIZE];
static uint8_t serialUartPort3RxBuf[BUF_SIZE];

static bool serial1_available();
static void serial1_putChar(char c);
static char serial1_getChar();
static void serial1_Printf(char *format, ...);

static bool serial2_available();
static void serial2_putChar(char c);
static char serial2_getChar();
static void serial2_Printf(char *format, ...);

static bool serial3_available();
static void serial3_putChar(char c);
static char serial3_getChar();
static void serial3_Printf(char *format, ...);

/****************************************
 * Internal Functions
 ****************************************/
/**
 * 시리얼/유아트 포트로 데이터를 송신
 * @param instance : 사용할 시리얼/유아트 포트
 * @param c : 송신할 데이터
 */
static void putchar_(serialUartPort_t* instance, char c) {
  if (instance->serialMode == SERIAL_INTERRPUT_MODE) {
    instance->txQueue.buf[instance->txQueue.head++] = c;
    instance->txQueue.head %= instance->txQueue.size;
    USART_ITConfig(instance->uartPort, USART_IT_TXE, ENABLE);
  }
  else if (instance->serialMode == SERIAL_POLLING_MODE) {
    USART_SendData(instance->uartPort, c);
    while (USART_GetFlagStatus(instance->uartPort, USART_FLAG_TXE) == RESET)
      ;
  }
}

/**
 * 시리얼/유아트 포트로 데이터를 수신
 * @param instance : 사용할 시리얼/유아트 포트
 * @return 수신된 데이터
 */
static char getchar_(serialUartPort_t* instance) {
  char c = '\0';
  if (instance->serialMode == SERIAL_INTERRPUT_MODE) {
    while (1) {
      if (instance->rxQueue.head != instance->rxQueue.tail) {
        c = instance->rxQueue.buf[instance->rxQueue.tail++];
        instance->rxQueue.tail %= instance->rxQueue.size;
      }
//    if (c != '\0') // wait when input data via serial
      break;
    }
  }
  else if (instance->serialMode == SERIAL_POLLING_MODE) {
    char c = '\0';
    while (USART_GetFlagStatus(instance->uartPort, USART_FLAG_RXNE) == RESET)
      ;
    c = (char) USART_ReceiveData(instance->uartPort);
  }
  return c;
}

/**
 * 시리얼/유아트 포트 초기화
 * @param instance : 사용할 시리얼/유아트 포트
 */
static void serialUartOpen(serialUartPort_t* instance) {
  if (instance->serialMode == SERIAL_INTERRPUT_MODE || instance->serialMode == SERIAL_POLLING_MODE) {
    RCC_APB2PeriphClockCmd(instance->rx_gpioClock, ENABLE);
    RCC_APB2PeriphClockCmd(instance->tx_gpioClock, ENABLE);

    if (instance->uartClock != RCC_APB2Periph_USART1)
      RCC_APB1PeriphClockCmd(instance->uartClock, ENABLE); // 2345
    else
      RCC_APB2PeriphClockCmd(instance->uartClock, ENABLE); // 1

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = instance->tx_gpioPin; // tx pa9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(instance->tx_gpioPort, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = instance->rx_gpioPin; // rx pa10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(instance->rx_gpioPort, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = instance->uartBaudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = instance->uartStopbit;
    USART_InitStructure.USART_Parity = instance->uartParity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(instance->uartPort, &USART_InitStructure);

    if (instance->serialMode == SERIAL_INTERRPUT_MODE) {
      USART_ITConfig(instance->uartPort, USART_IT_RXNE, ENABLE);

      NVIC_InitTypeDef NVIC_InitStructure;
      NVIC_InitStructure.NVIC_IRQChannel = instance->uartIRQ;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
    }

    USART_Cmd(instance->uartPort, ENABLE);
  }
  else {
    // SOFT SERIAL
  }
}

/****************************************
 * External Functions
 ****************************************/
/**
 * openSerial1
 * @note Rx : PA10, Tx: PA9, BaudRate : 115200
 * @param mode : INTERRUPT_MODE, SERIAL_POLLING_MODE
 * @return serialUartPort_t*
 */
serialUartPort_t* openSerial1(serialMode_t mode) {
  serialUartPort1.rx_gpioClock = RCC_APB2Periph_GPIOA;
  serialUartPort1.rx_gpioPin = GPIO_Pin_10;
  serialUartPort1.rx_gpioPort = GPIOA;

  serialUartPort1.tx_gpioClock = RCC_APB2Periph_GPIOA;
  serialUartPort1.tx_gpioPin = GPIO_Pin_9;
  serialUartPort1.tx_gpioPort = GPIOA;

  serialUartPort1.uartClock = RCC_APB2Periph_USART1;
  serialUartPort1.uartPort = USART1;
  serialUartPort1.uartIRQ = USART1_IRQn;

  serialUartPort1.uartBaudrate = 115200;
  serialUartPort1.uartStopbit = USART_StopBits_1;
  serialUartPort1.uartParity = USART_Parity_No;

  serialUartPort1.getChar = serial1_getChar;
  serialUartPort1.putChar = serial1_putChar;
  serialUartPort1.printf = serial1_Printf;
  serialUartPort1.available = serial1_available;

  serialUartPort1.serialMode = mode;

  serialUartPort1.txQueue.buf = serialUartPort1TxBuf;
  serialUartPort1.txQueue.size = BUF_SIZE;
  serialUartPort1.rxQueue.buf = serialUartPort1RxBuf;
  serialUartPort1.rxQueue.size = BUF_SIZE;

  serialUartOpen(&serialUartPort1);

  return &serialUartPort1;
}
/**
 * 시리얼1 포트에 수신 버퍼에 데이터가 있는지 확인
 * @return 데이터의 유무
 */
static bool serial1_available() {
  return serialUartPort1.rxQueue.head != serialUartPort1.rxQueue.tail;
}
/**
 * 시리얼1 포트로 1바이트 데이터 송신
 * @param c 송신할 데이터
 */
static void serial1_putChar(char c) {
  putchar_(&serialUartPort1, c);
}
/**
 * 시리얼1 포트로 1바이트 데이터 수신
 * @return 수신된 데이터
 */
static char serial1_getChar() {
  return getchar_(&serialUartPort1);
}
/**
 * 시리얼1 포트로 1바이트 데이터 송신(Printf에서 사용될 함수)
 * @param p 사용안함
 * @param c 전송할 데이터
 */
static void serial1_putc(void *p, char c) {
  putchar_(&serialUartPort1, c);
}
/**
 * 시리얼1 포트로 문자열 송신
 * @param format 전송할 문자열과 포맷
 */
static void serial1_Printf(char *format, ...) {
  va_list va;
  va_start(va, format);
  tfp_format(NULL, serial1_putc, format, va);
  va_end(va);
}
/**
 * openSerial2
 * @note Rx : PA3, Tx: PA2, BaudRate : 115200
 * @param mode : INTERRUPT_MODE, SERIAL_POLLING_MODE
 * @return serialUartPort_t*
 */
serialUartPort_t* openSerial2(serialMode_t mode) {
  serialUartPort2.rx_gpioClock = RCC_APB2Periph_GPIOA;
  serialUartPort2.rx_gpioPin = GPIO_Pin_3;
  serialUartPort2.rx_gpioPort = GPIOA;

  serialUartPort2.tx_gpioClock = RCC_APB2Periph_GPIOA;
  serialUartPort2.tx_gpioPin = GPIO_Pin_2;
  serialUartPort2.tx_gpioPort = GPIOA;

  serialUartPort2.uartClock = RCC_APB2Periph_USART1;
  serialUartPort2.uartPort = USART2;
  serialUartPort2.uartIRQ = USART2_IRQn;

  serialUartPort2.uartBaudrate = 115200;
  serialUartPort2.uartStopbit = USART_StopBits_1;
  serialUartPort2.uartParity = USART_Parity_No;

  serialUartPort2.getChar = serial2_getChar;
  serialUartPort2.putChar = serial2_putChar;
  serialUartPort2.printf = serial2_Printf;
  serialUartPort2.available = serial2_available;

  serialUartPort2.serialMode = mode;

  serialUartPort2.txQueue.buf = serialUartPort2TxBuf;
  serialUartPort2.txQueue.size = BUF_SIZE;
  serialUartPort2.rxQueue.buf = serialUartPort2RxBuf;
  serialUartPort2.rxQueue.size = BUF_SIZE;

  serialUartOpen(&serialUartPort2);

  return &serialUartPort2;
}
static bool serial2_available() {
  return serialUartPort2.rxQueue.head != serialUartPort2.rxQueue.tail;
}
static void serial2_putChar(char c) {
  putchar_(&serialUartPort2, c);
}
static char serial2_getChar() {
  return getchar_(&serialUartPort2);
}
static void serial2_putc(void *p, char c) {
  putchar_(&serialUartPort2, c);
}
static void serial2_Printf(char *format, ...) {
  va_list va;
  va_start(va, format);
  tfp_format(NULL, serial2_putc, format, va);
  va_end(va);
}
/**
 * openSerial3
 * @note Rx : PB11, Tx: PB10, BaudRate : 115200
 * @param mode : INTERRUPT_MODE, SERIAL_POLLING_MODE
 * @return serialUartPort_t*
 */
serialUartPort_t* openSerial3(serialMode_t mode) {
  serialUartPort3.rx_gpioClock = RCC_APB2Periph_GPIOB;
  serialUartPort3.rx_gpioPin = GPIO_Pin_11;
  serialUartPort3.rx_gpioPort = GPIOB;

  serialUartPort3.tx_gpioClock = RCC_APB2Periph_GPIOB;
  serialUartPort3.tx_gpioPin = GPIO_Pin_10;
  serialUartPort3.tx_gpioPort = GPIOB;

  serialUartPort3.uartClock = RCC_APB1Periph_USART3;
  serialUartPort3.uartPort = USART3;
  serialUartPort3.uartIRQ = USART3_IRQn;

  serialUartPort3.uartBaudrate = 115200;
  serialUartPort3.uartStopbit = USART_StopBits_1;
  serialUartPort3.uartParity = USART_Parity_No;

  serialUartPort3.getChar = serial3_getChar;
  serialUartPort3.putChar = serial3_putChar;
  serialUartPort3.printf = serial3_Printf;
  serialUartPort3.available = serial3_available;

  serialUartPort3.serialMode = mode;

  serialUartPort3.txQueue.buf = serialUartPort3TxBuf;
  serialUartPort3.txQueue.size = BUF_SIZE;
  serialUartPort3.rxQueue.buf = serialUartPort3RxBuf;
  serialUartPort3.rxQueue.size = BUF_SIZE;

  serialUartOpen(&serialUartPort3);

  return &serialUartPort3;
}
static bool serial3_available() {
  return serialUartPort3.rxQueue.head != serialUartPort3.rxQueue.tail;
}
static void serial3_putChar(char c) {
  putchar_(&serialUartPort3, c);
}
static char serial3_getChar() {
  return getchar_(&serialUartPort3);
}
static void serial3_putc(void *p, char c) {
  putchar_(&serialUartPort3, c);
}
static void serial3_Printf(char *format, ...) {
  va_list va;
  va_start(va, format);
  tfp_format(NULL, serial3_putc, format, va);
  va_end(va);
}

/****************************************
 * Interrupt Handler
 ****************************************/
/**
 * 시리얼/유아트 포트 인터럽트 핸들러
 * @param instance 호출된 시리얼/유아트 포트
 */
static void serialUart_handler(serialUartPort_t* instance) {
  if (USART_GetITStatus(instance->uartPort, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(instance->uartPort, USART_IT_RXNE);
    instance->rxQueue.buf[instance->rxQueue.head++] = USART_ReceiveData(instance->uartPort);
    instance->rxQueue.head %= instance->rxQueue.size;
  }
  if (USART_GetITStatus(instance->uartPort, USART_IT_TXE) != RESET) {
    USART_ClearITPendingBit(instance->uartPort, USART_IT_TXE);
    USART_SendData(instance->uartPort, instance->txQueue.buf[instance->txQueue.tail++]);
    instance->txQueue.tail %= instance->txQueue.size;
    if (instance->txQueue.head == instance->txQueue.tail)
      USART_ITConfig(instance->uartPort, USART_IT_TXE, DISABLE);
  }
}

void USART1_IRQHandler() {
  serialUart_handler(&serialUartPort1);
}

void USART2_IRQHandler() {
  serialUart_handler(&serialUartPort2);
}

void USART3_IRQHandler() {
  serialUart_handler(&serialUartPort3);
}
