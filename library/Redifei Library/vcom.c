/*
 vcom.c - vcom library
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
#include <hw_config.h>
#include <usb_lib.h>
#include <usb_desc.h>
#include <usb_pwr.h>
#include <usb_istr.h>
#include "printf.h"
#include "vcom.h"

#define BUF_SIZE 256

typedef struct {
  uint8_t rxBuf[BUF_SIZE];
} red_vcomPortBuf_t;

static red_vcomPort_t* vcomPrintfPort;
static red_vcomPort_t vcomPorts[RED_VCOM_PORT_MAX];
static red_vcom_setting_t vcomSettings[RED_VCOM_PORT_MAX];
static red_vcomPortBuf_t vcomPortBuf[RED_VCOM_PORT_MAX];

// TODO: this exception functions to be deleted
// Those use when receive data for usb
void red_queuePush(struct red_vcomPort* this, char c) {
  this->setting->queue.buf[this->setting->queue.head++] = c;
  this->setting->queue.head %= this->setting->queue.size;
}
char red_queuePop(struct red_vcomPort* this) {
  char c = '\0';
  if (this->setting->queue.head != this->setting->queue.tail) {
    c = this->setting->queue.buf[this->setting->queue.tail++];
    this->setting->queue.tail %= this->setting->queue.size;
  }
  return c;
}

void vcomQueuePush_in_hwConfig(char c) {
  red_queuePush(&vcomPorts[RED_VCOM_PORT], c);
}

// FIXME: have error
static bool red_available(struct red_vcomPort* this) {
  return this->setting->queue.head != this->setting->queue.tail;
}
static void red_putChar(struct red_vcomPort* this, char c) {
// XXX: why used if?
//  if (bDeviceState == CONFIGURED) {
  USB_Send_Data(c);
//  }
}
static char red_getChar(struct red_vcomPort* this) {
  return red_queuePop(this);
}
static void red_putc(void *p, char c) {
  red_putChar(vcomPrintfPort, c);
}
static void red_Printf(struct red_vcomPort* this, char *format, ...) {
  vcomPrintfPort = this;
  va_list va;
  va_start(va, format);
  tfp_format(NULL, red_putc, format, va);
  va_end(va);
}

static void red_vcomConfig(struct red_vcomPort* this) {
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
}

red_vcomPort_t* redVcomInit(uint8_t vcomPortNum, red_vcom_userSetting_t* userSetting) {
  red_vcomPort_t* vcomPort = &vcomPorts[vcomPortNum];

  vcomSettings[vcomPortNum].hw = &redVcomWareMap[vcomPortNum];
  vcomSettings[vcomPortNum].userSetting = userSetting;
  vcomPort->setting = &vcomSettings[vcomPortNum];

  vcomPort->setting->queue.buf = vcomPortBuf[vcomPortNum].rxBuf;
  vcomPort->setting->queue.size = BUF_SIZE;

  vcomPort->getChar = red_getChar;
  vcomPort->putChar = red_putChar;
  vcomPort->printf = red_Printf;
  vcomPort->available = red_available;

  red_vcomConfig(vcomPort);
  return vcomPort;
}

// XXX: why don't use tx interrupt
void USB_LP_CAN1_RX0_IRQHandler() {
  USB_Istr();
}

void USBWakeUp_IRQHandler(void) {
  EXTI_ClearITPendingBit(EXTI_Line18);
}
