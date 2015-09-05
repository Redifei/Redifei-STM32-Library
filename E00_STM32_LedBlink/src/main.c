/*
 * Redifei: E00 LedBlink
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
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
