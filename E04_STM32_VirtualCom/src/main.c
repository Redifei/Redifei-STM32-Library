/*
 * Redifei: E04 Virtual Com Library Example
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 */
#include "stm32f10x_conf.h"
#include "printf.h"

#include "i2c.h"
#include "pwm.h"
#include "serial.h"
#include "vcom.h"

serialUartPort_t*   serialPort;
vComPort_t*         vcomPort;
i2cPort_t*          i2cPort;
timPwmPort_t*       pwmInPort;
timPwmPort_t*       pwmOutPort;


void _delay(uint32_t ms) {
  ms *= 8000;
  while(--ms);
}


int main(void) {
  // Open VCom Port
  vcomPort = openVCom();
  vcomPort->printf("Hello World!\r\n");

  while (1) {
    static uint8_t i;
    if(vcomPort->available()) {
      vcomPort->printf("%d : %c\r\n", i++, vcomPort->getChar());
    }
    _delay(10);
  }
}
