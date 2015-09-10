/*
 main.c - main for stm32_serial
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
#include <stm32f10x_conf.h>
#include "systickTimer.h"
#include "serialUart.h"
#include "i2c.h"
#include "timerPwm.h"
#include "vcom.h"

red_serialUartPort_t* redSerial;

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

  redSerial->printf(redSerial, "Hello World!!\r\n");
  delay(1000);

  // print input data via serial
  while (1) {
    static uint8_t i;
    if (redSerial->available(redSerial)) {
      redSerial->printf(redSerial, "%d : %c\r\n", i++, redSerial->getChar(redSerial));
    }
    delay(100);
  }
}
