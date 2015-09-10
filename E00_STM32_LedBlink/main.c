/*
 main.c - main for stm32_ledBlink
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
#include "stm32f10x_conf.h"

void _delay(volatile uint32_t ms) {
  ms *= 800;
  while(ms--);
}

int main(void) {
  // LedBlink
  // Config LED Pin(PC13)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC,&GPIO_InitStructure);

  while(1) {
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, SET);
    _delay(1000);
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, RESET);
    _delay(1000);
  }
}
