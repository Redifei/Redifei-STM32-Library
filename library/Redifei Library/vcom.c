/*
 * Redifei: Virtual Port Library
 * Based on Oroca's SkyRover Project
 *
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 *
 *
 * TODO:
 * Support USB TX interrupt
 * FIXME:
 * Can't read null
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

#include <stdlib.h>
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_istr.h"
#include "printf.h"
#include "vcom.h"

#define BUF_SIZE 256

static vComPort_t vComPort;

static uint8_t vComPortRxBuf[BUF_SIZE];

static char vCom_getChar();
static void vCom_putChar(char c);
static void vCom_Printf(char *format, ...);

static void vComOpen() {
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
}

vComPort_t* openVCom() {
  vComPort.queue.buf = vComPortRxBuf;
  vComPort.queue.size = BUF_SIZE;

  vComPort.getChar = vCom_getChar;
  vComPort.putChar = vCom_putChar;
  vComPort.printf = vCom_Printf;

  vComOpen();
  return &vComPort;
}

// Those use when receive data for usb
void vComPush(uint8_t c) {
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

// Read/Write
static char vCom_getChar() {
  return vComPop();
}

static void vCom_putChar(char c) {
// XXX: why used if?
//  if (bDeviceState == CONFIGURED) {
    USB_Send_Data(c);
//  }
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

// Redifei: ADD IRQ Handlers
// XXX: why don't use tx interrupt
void USB_LP_CAN1_RX0_IRQHandler() {
  USB_Istr();
}

void USBWakeUp_IRQHandler(void) {
  EXTI_ClearITPendingBit(EXTI_Line18);
}
