/*
 * Redifei: Virtual Port Library
 * Based on Oroca's SkyRover Project
 *
 ******************************************************************************
 * This file is part of Redifei Library.
 *
 * Redifei Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Redifei Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Redifei Library.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/*
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
 * ->available
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

#include <stdlib.h>
#include <hw_config.h>
#include <usb_lib.h>
#include <usb_desc.h>
#include <usb_pwr.h>
#include <usb_istr.h>
#include "printf.h"
#include "vcom.h"

#define BUF_SIZE 256

static vComPort_t vComPort;

static uint8_t vComPortRxBuf[BUF_SIZE];

static bool vCom_available();
static char vCom_getChar();
static void vCom_putChar(char c);
static void vCom_Printf(char *format, ...);

char vComPop();

/****************************************
 * Internal Functions
 ****************************************/
static void vComOpen() {
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
}

/****************************************
 * External Functions
 ****************************************/
/**
 * openVCom
 * @note D+ : PA12, D- : PA11, DISCONNECT_PIN : PC14
 * @return vComPort_t*
 */
vComPort_t* openVCom() {
  vComPort.queue.buf = vComPortRxBuf;
  vComPort.queue.size = BUF_SIZE;

  vComPort.getChar = vCom_getChar;
  vComPort.putChar = vCom_putChar;
  vComPort.printf = vCom_Printf;
  vComPort.available = vCom_available;

  vComOpen();
  return &vComPort;
}
static bool vCom_available() {
  return vComPort.queue.head != vComPort.queue.tail;
}
static void vCom_putChar(char c) {
// XXX: why used if?
//  if (bDeviceState == CONFIGURED) {
  USB_Send_Data(c);
//  }
}
static char vCom_getChar() {
  return vComPop();
}
static void vCom_putc(void *p, char c) {
  vCom_putChar(c);
}
static void vCom_Printf(char *format, ...) {
  va_list va;
  va_start(va, format);
  tfp_format(NULL, vCom_putc, format, va);
  va_end(va);
}

// TODO: this exception functions to be deleted
// Those use when receive data for usb
void vComPush(char c) {
  vComPort.queue.buf[vComPort.queue.head++] = c;
  vComPort.queue.head %= vComPort.queue.size;
}
char vComPop() {
  char c = '\0';
  if (vComPort.queue.head != vComPort.queue.tail) {
    c = vComPort.queue.buf[vComPort.queue.tail++];
    vComPort.queue.tail %= vComPort.queue.size;
  }
  return c;
}

/****************************************
 * Interrupt Handler
 ****************************************/
// XXX: why don't use tx interrupt
void USB_LP_CAN1_RX0_IRQHandler() {
  USB_Istr();
}

void USBWakeUp_IRQHandler(void) {
  EXTI_ClearITPendingBit(EXTI_Line18);
}
