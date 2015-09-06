/*
 * Redifei: E02_0 I2C Library Example
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
  // I2C Library Example
  // Config systemTick Timer
  systickTimerConfig();

  // Open Serial Port
  serialPort = openSerial3(SERIAL_INTERRPUT_MODE);
  serialPort->printf("Hello World!\r\n");

  // Open I2C Port
  i2cPort = openI2C1(I2C_INTERRPUT_MODE);
  serialPort->printf("Open I2C Port\r\n");

  i2cPort->write1Byte(0x68, 0x6B, 0x80);
  delay(5);

  i2cPort->write1Byte(0x68, 0x6B, 0x03);
  serialPort->printf("I2C Device Init Done\r\n");
  delay(500);

  while (1) {
    uint8_t buf[2], err;

    // Read Mpu6050 Acc Z axis
    err = i2cPort->readBytes(0x68, 0x3F, 2, buf);
    serialPort->printf("%d | %d\r\n", (int16_t)(buf[0] << 8 | buf[1]), err);
    delay(100);
  }
}
