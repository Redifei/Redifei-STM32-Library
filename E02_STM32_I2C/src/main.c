/*
 * Redifei: E02 I2C Library Example
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
  // I2C Library Example
  // Open Serial Port
  serialPort = openSerial3(SERIAL_INTERRPUT_MODE);
  serialPort->printf("Hello World!\r\n");

  // Open I2C Port
  i2cPort = openI2C1(I2C_POLLING_MODE);
  serialPort->printf("Open I2C Port\r\n");

  i2cPort->write1Byte(0x68, 0x6B, 0x80);
  _delay(5);

  i2cPort->write1Byte(0x68, 0x6B, 0x03);
  serialPort->printf("I2C Device Init Done\r\n");
  _delay(500);

  while(1) {
    uint8_t buf[2], err;

    // Read Mpu6050 Acc Z axis
    err = i2cPort->readBytes(0x68, 0x3F, 2, buf);
    serialPort->printf("%d | %d\r\n", (int16_t)(buf[0] << 8 | buf[1]), err);
    _delay(100);
  }
}
