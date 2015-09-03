/*
 * Redifei: E04 Virtual Com Library Example
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
  // Virtual Com Library Example
  // Config systemTick Timer
  systickTimerConfig();

  // Open VCom Port
  vcomPort = openVCom();
  vcomPort->printf("Hello World!\r\n");

  while (1) {
    static uint8_t i;
    if (vcomPort->available()) {
      vcomPort->printf("%d : %c\r\n", i++, vcomPort->getChar());
    }
    delay(10);
  }
}
