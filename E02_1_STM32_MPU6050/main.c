/*
 * WILL BE CHANGE TO VERSION2 LIBRARY
 *
 *
 * Redifei: E02_1 MPU6050 Library Example
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
#include <stdlib.h>
#include <stm32f10x_conf.h>
#include "systickTimer.h"
#include "serialUart.h"
#include "i2c.h"
#include "timerPwm.h"
#include "vcom.h"

#include "mpu6050.h"

serialUartPort_t* redSerial;

void assert_failed(uint8_t* file, uint32_t line) {
  if(redSerial != NULL)
    redSerial->printf(redSerial, "Assert fail at File %s Line %d\r\n", file, (uint16_t)line);
  while(1);
}

int main(void) {
  systickTimerConfig();

  // Init SerialPort
  red_serialUart_userSetting_t redSerialUserSetting = {
      .serialMode = RED_SERIAL_POLLING_MODE,
      .baudrate = 115200,
      .parity = USART_Parity_No,
      .stopbit = USART_StopBits_1,
  };
  redSerial = redSerialUartInit(RED_SERIAL_UART_PORT_3, &redSerialUserSetting);

  // Config MPU6050
  red_mpu6050_userSetting_t mpu6050UserSetting = {
      .i2cPortNum = RED_I2C_PORT_1,
      .i2cMode = RED_I2C_INTERRPUT_MODE,
      .i2cClockSpeed = 400000,
      .i2cTimeout = I2C_DEFAULT_TIMEOUT,
  };
  red_mpu6050Port_t* redMpu6050 = redMpu6050Init(RED_MPU6050_1, &mpu6050UserSetting);

  redSerial->printf(redSerial, "Detected : 0x%x\r\n", myMPU->deviceAddr);
  delay(1000);

  while (1) {
    int16_t buf[3];

    // Read Mpu6050 Acc 3 axis
    bool err = redMpu6050->accRead(redMpu6050, buf);
    redSerial->printf(redSerial, "%d, %d, %d | %d\r\n", buf[0], buf[1], buf[2], err);
    delay(100);
  }
}
