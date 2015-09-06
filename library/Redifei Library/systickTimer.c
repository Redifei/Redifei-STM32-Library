/*
 * Redifei: systickTimer Library
 * Based on BaseFlight Project
 *
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

/*
 * FIXME(systickTimer):
 * micros countable is 70 Minute
 *
 *
 * -Functions
 * systickTimerConfig
 * micros
 * millis
 * delayMicroseconds
 * delay
 */

#include <stm32f10x_conf.h>
#include "systickTimer.h"

static volatile uint32_t systickCount;
static uint32_t usTicks;

/****************************************
 * External Functions
 ****************************************/

/**
 * systickTimerConfig
 * @note SysTick_Config : SYSCLK_Frequency / 1000
 */
void systickTimerConfig() {
  RCC_ClocksTypeDef RCC_CLOCKS;
  RCC_GetClocksFreq(&RCC_CLOCKS);
  while(SysTick_Config(RCC_CLOCKS.SYSCLK_Frequency / 1000)); // set period 1/1000s(1ms)
  usTicks = RCC_CLOCKS.SYSCLK_Frequency / 1000000;
}

// FIXME: micros countable is 70 Minute
uint32_t micros() {
  register uint32_t ms, cycleCount;
  do {
    ms = systickCount;
    cycleCount = SysTick->VAL;
  } while(ms != systickCount);
  return (ms*1000) + ((usTicks * 1000 - cycleCount) / usTicks);
}

// FIXME: micros countable is 50 Days
uint32_t millis() {
  return systickCount;
}

void delayMicroseconds(uint32_t us) {
  uint32_t now = micros();
  while(micros() - now < us);
}

void delay(uint32_t ms) {
  while(ms--) {
    delayMicroseconds(1000);
  }
}

/****************************************
 * Interrupt Handler
 ****************************************/
void SysTick_Handler() {
  systickCount++;
}
