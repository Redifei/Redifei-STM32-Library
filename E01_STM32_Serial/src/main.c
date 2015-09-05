/*
 * Redifei: E01 Serial Library Example
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 */
#include <stm32f10x_conf.h>

#include "systickTimer.h"
#include "i2c.h"
#include "pwm.h"
#include "printf.h"
#include "serial.h"
#include "vcom.h"

serialUartPort_t* serialPort;
vComPort_t* vcomPort;
i2cPort_t* i2cPort;
timPwmPort_t* pwmInPort;
timPwmPort_t* pwmOutPort;

int main(void) {
  // Serial Library Example
  // Config systemTick Timer
  systickTimerConfig();

  // Open Serial Port
  serialPort = openSerial3(SERIAL_INTERRPUT_MODE);
  serialPort->printf("Hello World!\r\n");

  while (1) {
    static uint8_t i;
    if (serialPort->available()) {
      serialPort->printf("%d : %c\r\n", i++, serialPort->getChar());
    }
    delay(10);
  }
}
