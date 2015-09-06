/*
 * Redifei: E03 TimerPWM Library Example
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
  // TimerPWM Library Example
  // Config systemTick Timer
  systickTimerConfig();

  // Open Serial Port
  serialPort = openSerial3(SERIAL_INTERRPUT_MODE);
  serialPort->printf("Hello World!\r\n");

  // Open TimerPWM
  pwmInPort = openTimerPwm101(TIM_PWM_INPUT_MODE);
  pwmOutPort = openTimerPwm301(TIM_PWM_OUTPUT_MODE);
  pwmOutPort = openTimerPwm302(TIM_PWM_OUTPUT_MODE);

  pwmOutPort->chan1Write(500);
  pwmOutPort->chan2Write(1500);

  while (1) {
    serialPort->printf("%d\r\n", pwmInPort->chan1Read());
    delay(100);
  }
}
