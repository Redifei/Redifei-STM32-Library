/*
 * Redifei: Timer Pwm I/O Library
 * Based on BaseFlight Project
 *
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 *
 *
 * FIXME:
 * Error PWM output channels : Timer3/2chan, Timer4/1chan, Timer4/2chan
 * TODO:
 * Not test timer2 all channel
 *
 *
 * -Functions
 * openTimerPwmx0y : x(1;4), y(1;4)
 * ->chanxWrite : x(1;4)
 * ->chanxRead : x(1;4)
 */

#pragma once

typedef enum {
  TIM_PWM_INPUT_MODE,
  TIM_PWM_OUTPUT_MODE,
} timPwmMode_t;

typedef struct {
  uint16_t        chan1_gpioPin;
  GPIO_TypeDef*   chan1_gpioPort;
  uint32_t        chan1_gpioClock;

  uint16_t        chan2_gpioPin;
  GPIO_TypeDef*   chan2_gpioPort;
  uint32_t        chan2_gpioClock;

  uint16_t        chan3_gpioPin;
  GPIO_TypeDef*   chan3_gpioPort;
  uint32_t        chan3_gpioClock;

  uint16_t        chan4_gpioPin;
  GPIO_TypeDef*   chan4_gpioPort;
  uint32_t        chan4_gpioClock;

  /* ALL CHANNEL SAME*/
  TIM_TypeDef*    timPort;
  uint32_t        timClock;
  IRQn_Type       timIRQ;

  uint32_t resolution;
  uint16_t hz;

  timPwmMode_t timMode;

  /* IRQ */
  uint16_t riseTime[4];
  uint16_t fallTime[4];
  uint16_t duty[4];
  uint8_t state[4]; // bool

  void (*chan1Write) (uint16_t duty);
  void (*chan2Write) (uint16_t duty);
  void (*chan3Write) (uint16_t duty);
  void (*chan4Write) (uint16_t duty);

  uint16_t (*chan1Read) ();
  uint16_t (*chan2Read) ();
  uint16_t (*chan3Read) ();
  uint16_t (*chan4Read) ();
}timPwmPort_t;

timPwmPort_t* openTimerPwm101(timPwmMode_t mode);
timPwmPort_t* openTimerPwm102(timPwmMode_t mode);
timPwmPort_t* openTimerPwm103(timPwmMode_t mode);
timPwmPort_t* openTimerPwm104(timPwmMode_t mode);

timPwmPort_t* openTimerPwm201(timPwmMode_t mode);
timPwmPort_t* openTimerPwm202(timPwmMode_t mode);
timPwmPort_t* openTimerPwm203(timPwmMode_t mode);
timPwmPort_t* openTimerPwm204(timPwmMode_t mode);

timPwmPort_t* openTimerPwm301(timPwmMode_t mode);
timPwmPort_t* openTimerPwm302(timPwmMode_t mode);
timPwmPort_t* openTimerPwm303(timPwmMode_t mode);
timPwmPort_t* openTimerPwm304(timPwmMode_t mode);

timPwmPort_t* openTimerPwm401(timPwmMode_t mode);
timPwmPort_t* openTimerPwm402(timPwmMode_t mode);
timPwmPort_t* openTimerPwm403(timPwmMode_t mode);
timPwmPort_t* openTimerPwm404(timPwmMode_t mode);
