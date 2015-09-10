/*
 main.c - main for stm32_timerPwm
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

void callback(uint8_t chan) {

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

  // Init TimerPwm Input
  red_timPwm_userSetting_t redTimerPwmInputUserSetting = {
      .timPwmMode = RED_TIMER_PWM_INPUT_MODE,
      .resolution = 1000000,
      .callback = callback,
  };
  red_timPwmPort_t* redTimerPwmIn1 = redTimerPwmInit(RED_TIMER1_PWM_CHANNEL_1, &redTimerPwmInputUserSetting);

  // Init TimerPwm Output
  red_timPwm_userSetting_t redTimerPwmOutPutUserSetting = {
      .timPwmMode = RED_TIMER_PWM_OUTPUT_MODE,
      .resolution = 1000000,
      .hz = 250,
  };
  red_timPwmPort_t* redTimerPwmOut1 = redTimerPwmInit(RED_TIMER3_PWM_CHANNEL_1, &redTimerPwmOutPutUserSetting);
  redTimerPwmOut1->write(redTimerPwmOut1, 1234);

  redSerial->printf(redSerial, "Hello World!!\r\n");
  delay(1000);

  while (1) {
    redSerial->printf(redSerial, "%d\r\n", redTimerPwmIn4->read(redTimerPwmIn1));
    delay(100);
  }
}
