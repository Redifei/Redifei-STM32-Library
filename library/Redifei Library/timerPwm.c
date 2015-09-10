/*
 timerPwm.c - timerPwm library
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
#include <stdbool.h>
#include <stm32f10x_conf.h>
#include "timerPwm.h"

#include "serialUart.h"

extern red_serialUartPort_t* redSerial;

static red_timPwmPort_t timPwmPorts[RED_TIMER_PWM_CHANNEL_MAX] = { NULL, };
static red_timPwm_setting_t timPwmSettings[RED_TIMER_PWM_CHANNEL_MAX] = { NULL, };

/**
 * red_timeBaseConfig
 * @note timer port setting (period, prescaler, ..)
 * @param this : red_timPwmPort object
 */
void red_timeBaseConfig(struct red_timPwmPort* this) {
  assert_param(this->setting != NULL);

  const red_timPwm_hardware_t* timerPwmHW = this->setting->hw;
  red_timPwm_userSetting_t* timerPwmSetting = this->setting->userSetting;

  // Clock Init, Calc Timer Clock Speed
  RCC_ClocksTypeDef RCC_CLOCKS;
  RCC_GetClocksFreq(&RCC_CLOCKS);

  uint32_t timerClocks;
  if (!(timerPwmHW->timClock & (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8))) {
    timerClocks = RCC_CLOCKS.PCLK1_Frequency * 2;
    RCC_APB1PeriphClockCmd(timerPwmHW->timClock, ENABLE);
  }
  else {
    timerClocks = RCC_CLOCKS.PCLK2_Frequency;
    RCC_APB2PeriphClockCmd(timerPwmHW->timClock, ENABLE);
  }

  // Timer TimeBase Init
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  if (timerPwmSetting->timPwmMode == RED_TIMER_PWM_INPUT_MODE)
    TIM_TimeBaseStructure.TIM_Period = 0xffff;
  else
    TIM_TimeBaseStructure.TIM_Period = timerPwmSetting->resolution / timerPwmSetting->hz - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = timerClocks / timerPwmSetting->resolution - 1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(timerPwmHW->timPort, &TIM_TimeBaseStructure);
}

/**
 * red_timPwmOutPortConfig
 * @param this : red_timPwmPort object
 */
void red_timPwmOutPortConfig(struct red_timPwmPort* this) {
  assert_param(this->setting != NULL);
  const red_timPwm_hardware_t* timerPwmHW = this->setting->hw;
  red_timPwm_userSetting_t* timerPwmSetting = this->setting->userSetting;
  assert_param(timerPwmSetting->timPwmMode == RED_TIMER_PWM_OUTPUT_MODE);

  TIM_TypeDef* timx = timerPwmHW->timPort;

  // GPIO Clock Init
  RCC_APB2PeriphClockCmd(timerPwmHW->gpioClock, ENABLE);

  // GPIO Init
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = timerPwmHW->gpioPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(timerPwmHW->gpioPort, &GPIO_InitStructure);

  // Timer Output Init
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  // only Advanced Timer
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

  switch (timerPwmHW->timChannel) {
  case TIM_Channel_1:
    TIM_OC1Init(timx, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(timx, TIM_OCPreload_Enable);
    break;
  case TIM_Channel_2:
    TIM_OC2Init(timx, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(timx, TIM_OCPreload_Enable);
    break;
  case TIM_Channel_3:
    TIM_OC3Init(timx, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(timx, TIM_OCPreload_Enable);
    break;
  case TIM_Channel_4:
    TIM_OC4Init(timx, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(timx, TIM_OCPreload_Enable);
    break;
  }

  // only Advenced Timer(TIM1, TIM8, ...)
  if (timerPwmHW->timClock & (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8))
    TIM_CtrlPWMOutputs(timx, ENABLE);

  TIM_Cmd(timx, ENABLE);
}

/**
 * red_timPwmInPortConfig
 * @param this : red_timPwmPort object
 */
void red_timPwmInPortConfig(struct red_timPwmPort* this) {
  assert_param(this->setting != NULL);
  const red_timPwm_hardware_t* timerPwmHW = this->setting->hw;
  red_timPwm_userSetting_t* timerPwmSetting = this->setting->userSetting;
  assert_param(timerPwmSetting->timPwmMode == RED_TIMER_PWM_INPUT_MODE);

  TIM_TypeDef* timx = timerPwmHW->timPort;

  // GPIO Clock Init
  RCC_APB2PeriphClockCmd(timerPwmHW->gpioClock, ENABLE);

  // GPIO Init
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = timerPwmHW->gpioPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(timerPwmHW->gpioPort, &GPIO_InitStructure);

  // Timer Input Init
  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = timerPwmHW->timChannel;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(timx, &TIM_ICInitStructure);

  switch (timerPwmHW->timChannel) {
  case TIM_Channel_1:
    TIM_ITConfig(timx, TIM_IT_CC1, ENABLE);
    break;
  case TIM_Channel_2:
    TIM_ITConfig(timx, TIM_IT_CC2, ENABLE);
    break;
  case TIM_Channel_3:
    TIM_ITConfig(timx, TIM_IT_CC3, ENABLE);
    break;
  case TIM_Channel_4:
    TIM_ITConfig(timx, TIM_IT_CC4, ENABLE);
    break;
  }

  // Timer IRQ Init
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = timerPwmHW->timIRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_Cmd(timx, ENABLE);
}

/**
 * red_timePwmOutWrite
 * @param this : red_timPwmPort object
 * @param duty : write pwm out duty
 */
void red_timePwmOutWrite(struct red_timPwmPort* this, uint16_t duty) {
  assert_param(this->setting != NULL);
  const red_timPwm_hardware_t* timerPwmHW = this->setting->hw;
  red_timPwm_userSetting_t* timerPwmSetting = this->setting->userSetting;
  assert_param(timerPwmSetting->timPwmMode == RED_TIMER_PWM_OUTPUT_MODE);

  // only Output Mode
  if(timerPwmSetting->timPwmMode != RED_TIMER_PWM_OUTPUT_MODE)
    return;

  TIM_TypeDef* timx = timerPwmHW->timPort;

  duty %= (timerPwmSetting->resolution / timerPwmSetting->hz); // resolution/hz = (maxValue)

  // write pwm duty
  switch (timerPwmHW->timChannel) {
  case TIM_Channel_1:
    TIM_SetCompare1(timx, duty);
    break;
  case TIM_Channel_2:
    TIM_SetCompare2(timx, duty);
    break;
  case TIM_Channel_3:
    TIM_SetCompare3(timx, duty);
    break;
  case TIM_Channel_4:
    TIM_SetCompare4(timx, duty);
    break;
  }
}

/**
 * red_timePwmInRead
 * @param this : red_timPwmPort object
 * @return read pwm in duty
 */
uint16_t red_timePwmInRead(struct red_timPwmPort* this) {
  assert_param(this->setting != NULL);
  red_timPwm_userSetting_t* timerPwmSetting = this->setting->userSetting;
  assert_param(timerPwmSetting->timPwmMode == RED_TIMER_PWM_INPUT_MODE);
  // read pwm duty
  return this->setting->duty;
}

/**
 * redTimerPwmInit
 * @param timPwmPortNum : RED_TIMERx_PWM_CHANNEL_y (x[1;4], y[1;4])
 * @param userSetting : timPwmMode : RED_TIMER_PWM_INPUT_MODE, RED_TIMER_PWM_OUTPUT_MODE
 *                      resolution : Timer Pwm duty resolution (us)
 *                      hz         : Timer Pwm duty hz (Output Only)
 *                      callback   : Timer Pwm IRQ Callback Function (Input Only)
 * @return red_timPwmPort object
 */
red_timPwmPort_t* redTimerPwmInit(uint8_t timPwmPortNum, red_timPwm_userSetting_t* userSetting) {
  red_timPwmPort_t* timPwmPort = &timPwmPorts[timPwmPortNum];

  timPwmSettings[timPwmPortNum].hw = &redTimerPwmHardWareMap[timPwmPortNum];
  timPwmSettings[timPwmPortNum].userSetting = userSetting;
  timPwmPort->setting = &timPwmSettings[timPwmPortNum];

  timPwmPort->setting->duty = 0;

  timPwmPort->write = red_timePwmOutWrite;
  timPwmPort->read = red_timePwmInRead;

  red_timeBaseConfig(timPwmPort);

  switch (timPwmPort->setting->userSetting->timPwmMode) {
  case RED_TIMER_PWM_INPUT_MODE:
    red_timPwmInPortConfig(timPwmPort);
    break;

  case RED_TIMER_PWM_OUTPUT_MODE:
    red_timPwmOutPortConfig(timPwmPort);
    break;
  }

  return timPwmPort;
}

/**
 * red_timer_Handler
 * @param this : red_timPwmPort object
 */
void red_timer_Handler(struct red_timPwmPort* this) {
  // only set timer to Input mode
  if(this->setting == NULL)
    return;
  if(this->setting->userSetting->timPwmMode != RED_TIMER_PWM_INPUT_MODE)
    return;

  const red_timPwm_hardware_t* timerPwmHW = this->setting->hw;
  red_timPwm_userSetting_t* timerPwmSetting = this->setting->userSetting;
  TIM_TypeDef* timx = timerPwmHW->timPort;

  // Check timer interrupt flag, read timer capture
  uint16_t capture, tim_ccer_ccnp;
  uint8_t channelNum;

  if ((TIM_GetITStatus(timx, TIM_IT_CC1) != RESET) && timerPwmHW->timChannel == TIM_Channel_1) {
    TIM_ClearITPendingBit(timx, TIM_IT_CC1);
    capture = TIM_GetCapture1(timx);
    tim_ccer_ccnp = TIM_CCER_CC1P;
    channelNum = 1;
  }
  else if ((TIM_GetITStatus(timx, TIM_IT_CC2) != RESET) && timerPwmHW->timChannel == TIM_Channel_2) {
    TIM_ClearITPendingBit(timx, TIM_IT_CC2);
    capture = TIM_GetCapture2(timx);
    tim_ccer_ccnp = TIM_CCER_CC2P;
    channelNum = 2;
  }
  else if ((TIM_GetITStatus(timx, TIM_IT_CC3) != RESET) && timerPwmHW->timChannel == TIM_Channel_3) {
    TIM_ClearITPendingBit(timx, TIM_IT_CC3);
    capture = TIM_GetCapture3(timx);
    tim_ccer_ccnp = TIM_CCER_CC3P;
    channelNum = 3;
  }
  else if ((TIM_GetITStatus(timx, TIM_IT_CC4) != RESET) && timerPwmHW->timChannel == TIM_Channel_4) {
    TIM_ClearITPendingBit(timx, TIM_IT_CC4);
    capture = TIM_GetCapture4(timx);
    tim_ccer_ccnp = TIM_CCER_CC4P;
    channelNum = 4;
  }
  else
    return; // Error?

  // calc Timer pwm duty
  if (this->setting->state == 0) {
    this->setting->riseTime = capture;
    this->setting->state = 1;
    timx->CCER |= tim_ccer_ccnp;
  }
  else {
    this->setting->fallTime = capture;
    this->setting->state = 0;
    this->setting->duty = this->setting->fallTime - this->setting->riseTime;
    timx->CCER &= ~tim_ccer_ccnp;
  }

  // if set callback function, call function
  if(timerPwmSetting->callback != NULL && (channelNum >= 1 && channelNum <= 4))
    timerPwmSetting->callback(channelNum);
}

void TIM1_CC_IRQHandler() {
  red_timer_Handler(&timPwmPorts[RED_TIMER1_PWM_CHANNEL_1]);
  red_timer_Handler(&timPwmPorts[RED_TIMER1_PWM_CHANNEL_2]);
  red_timer_Handler(&timPwmPorts[RED_TIMER1_PWM_CHANNEL_3]);
  red_timer_Handler(&timPwmPorts[RED_TIMER1_PWM_CHANNEL_4]);
}

void TIM2_IRQHandler() {
  red_timer_Handler(&timPwmPorts[RED_TIMER2_PWM_CHANNEL_1]);
  red_timer_Handler(&timPwmPorts[RED_TIMER2_PWM_CHANNEL_2]);
  red_timer_Handler(&timPwmPorts[RED_TIMER2_PWM_CHANNEL_3]);
  red_timer_Handler(&timPwmPorts[RED_TIMER2_PWM_CHANNEL_4]);
}

void TIM3_IRQHandler() {
  red_timer_Handler(&timPwmPorts[RED_TIMER3_PWM_CHANNEL_1]);
  red_timer_Handler(&timPwmPorts[RED_TIMER3_PWM_CHANNEL_2]);
  red_timer_Handler(&timPwmPorts[RED_TIMER3_PWM_CHANNEL_3]);
  red_timer_Handler(&timPwmPorts[RED_TIMER3_PWM_CHANNEL_4]);
}

void TIM4_IRQHandler() {
  red_timer_Handler(&timPwmPorts[RED_TIMER4_PWM_CHANNEL_1]);
  red_timer_Handler(&timPwmPorts[RED_TIMER4_PWM_CHANNEL_2]);
  red_timer_Handler(&timPwmPorts[RED_TIMER4_PWM_CHANNEL_3]);
  red_timer_Handler(&timPwmPorts[RED_TIMER4_PWM_CHANNEL_4]);
}
