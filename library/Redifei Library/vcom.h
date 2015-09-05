/*
 * Redifei: Virtual Port Library
 * Based on Oroca's SkyRover Project
 *
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 *
 *
 * TODO(VCOM):
 * Support USB TX interrupt
 * FIXME(VCOM):
 * Done : Can't read null
 *
 *
 * -Functions
 * openVCom
 * ->putChar
 * ->getChar
 * ->printf
 *
 * - Please set this value
 * platform_config.h : should configure USB_DISCONNECT_PIN
 * usb_desc.c : should configure vcom product, vcom vender
 *
 * - Changed with Base Example
 * platform_config.h : Include header files
 * hw_config.c : Add usb send data function, Delete usart configurations
 * usb_conf.h : Change imr mask
 * usb_prop.c : Delete usart configurations
 */

#pragma once

#include "queue.h"

typedef struct {
  /* 가상 컴포트에서 사용할 큐 */
  Qtype_t queue;

  /* 가상 컴포트에서 사용할 함수 */
  void (*putChar)(char);
  char (*getChar)();
  void (*printf)(char*, ...);
  bool (*available)();
} vComPort_t;

void vComPush(char c); // This Function only use in hw_config.c
vComPort_t* openVCom();
