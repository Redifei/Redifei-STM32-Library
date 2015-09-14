/*
 timerPwm.h - timerPwm library
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

#pragma once

enum {
  RED_TIMER1_PWM_CHANNEL_1,
  RED_TIMER1_PWM_CHANNEL_2,
  RED_TIMER1_PWM_CHANNEL_3,
  RED_TIMER1_PWM_CHANNEL_4,

  RED_TIMER2_PWM_CHANNEL_1,
  RED_TIMER2_PWM_CHANNEL_2,
  RED_TIMER2_PWM_CHANNEL_3,
  RED_TIMER2_PWM_CHANNEL_4,

  RED_TIMER3_PWM_CHANNEL_1,
  RED_TIMER3_PWM_CHANNEL_2,
  RED_TIMER3_PWM_CHANNEL_3,
  RED_TIMER3_PWM_CHANNEL_4,

  RED_TIMER4_PWM_CHANNEL_1,
  RED_TIMER4_PWM_CHANNEL_2,
  RED_TIMER4_PWM_CHANNEL_3,
  RED_TIMER4_PWM_CHANNEL_4,

  RED_TIMER_PWM_CHANNEL_MAX,
} red_TimerPwmDevices;

typedef enum {
  RED_TIMER_PWM_INPUT_MODE, RED_TIMER_PWM_OUTPUT_MODE,
} red_timPwmMode_t;

typedef struct {
  GPIO_TypeDef* gpioPort;
  uint32_t gpioClock;
  uint16_t gpioPin;

  TIM_TypeDef* timPort;
  uint32_t timClock;

  uint16_t timChannel;

  IRQn_Type timIRQ;
} red_timPwm_hardware_t;

static const red_timPwm_hardware_t redTimerPwmHardWareMap[RED_TIMER_PWM_CHANNEL_MAX] = {
    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_8, TIM1, RCC_APB2Periph_TIM1, TIM_Channel_1, TIM1_CC_IRQn },
    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_9, TIM1, RCC_APB2Periph_TIM1, TIM_Channel_2, TIM1_CC_IRQn },
    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_10, TIM1, RCC_APB2Periph_TIM1, TIM_Channel_3, TIM1_CC_IRQn },
    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_11, TIM1, RCC_APB2Periph_TIM1, TIM_Channel_4, TIM1_CC_IRQn },

    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_0, TIM2, RCC_APB1Periph_TIM2, TIM_Channel_1, TIM2_IRQn },
    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_1, TIM2, RCC_APB1Periph_TIM2, TIM_Channel_2, TIM2_IRQn },
    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_2, TIM2, RCC_APB1Periph_TIM2, TIM_Channel_3, TIM2_IRQn },
    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_3, TIM2, RCC_APB1Periph_TIM2, TIM_Channel_4, TIM2_IRQn },

    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_6, TIM3, RCC_APB1Periph_TIM3, TIM_Channel_1, TIM3_IRQn },
    { GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_7, TIM3, RCC_APB1Periph_TIM3, TIM_Channel_2, TIM3_IRQn },
    { GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_0, TIM3, RCC_APB1Periph_TIM3, TIM_Channel_3, TIM3_IRQn },
    { GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_1, TIM3, RCC_APB1Periph_TIM3, TIM_Channel_4, TIM3_IRQn },

    { GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_6, TIM4, RCC_APB1Periph_TIM4, TIM_Channel_1, TIM4_IRQn },
    { GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_7, TIM4, RCC_APB1Periph_TIM4, TIM_Channel_2, TIM4_IRQn },
    { GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_8, TIM4, RCC_APB1Periph_TIM4, TIM_Channel_3, TIM4_IRQn },
    { GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_9, TIM4, RCC_APB1Periph_TIM4, TIM_Channel_4, TIM4_IRQn },
};

typedef struct {
  red_timPwmMode_t timPwmMode;
  uint32_t resolution;
  uint16_t hz;
  void (*callback)(uint8_t chan);
} red_timPwm_userSetting_t;

typedef struct {
  const red_timPwm_hardware_t* hw;
  red_timPwm_userSetting_t* userSetting;
  uint16_t riseTime;
  uint16_t fallTime;
  uint16_t duty;
  uint8_t state; // bool
} red_timPwm_param_t;

typedef struct red_timPwmDevice {
  red_timPwm_param_t* param;
  void (*write)(struct red_timPwmDevice* this, uint16_t duty);
  uint16_t (*read)(struct red_timPwmDevice* this);
} red_timPwmDevice_t;

#define IS_CONFIGED_TIMER_PORT(THIS_SETTING) (THIS_SETTING != NULL)
#define IS_INPUT_TIMER_PORT(TIMER_PORT) (TIMER_PORT == RED_TIMER_PWM_INPUT_MODE)
#define IS_OUTPUT_TIMER_PORT(TIMER_PORT) (TIMER_PORT == RED_TIMER_PWM_OUTPUT_MODE)
#define IS_VAILD_TIMER_PORT_NUM(TIMER_NUM) (TIMER_NUM < RED_TIMER_PWM_CHANNEL_MAX)

red_timPwmDevice_t* redTimerPwmInit(uint8_t timPwmPortNum, red_timPwm_userSetting_t* userSetting);
