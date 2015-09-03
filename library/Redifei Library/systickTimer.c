/*
 * Redifei: systickTimer Library
 * Based on BaseFlight Project
 *
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 *
 *
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

void SysTick_Handler() {
  systickCount++;
}
