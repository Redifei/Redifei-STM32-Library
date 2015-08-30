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

#include "stm32f10x_conf.h"
#include "pwm.h"

timPwmPort_t timPwm1;
timPwmPort_t timPwm2;
timPwmPort_t timPwm3;
timPwmPort_t timPwm4;

void timeBaseInit(timPwmPort_t* instance, timPwmMode_t mode) {
  RCC_ClocksTypeDef RCC_CLOCKS;
  RCC_GetClocksFreq(&RCC_CLOCKS);

  uint32_t timerClocks;
  if(!(instance->timClock & (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8))) {
    timerClocks = RCC_CLOCKS.PCLK1_Frequency *2;
    RCC_APB1PeriphClockCmd(instance->timClock, ENABLE);
  }
  else {
    timerClocks = RCC_CLOCKS.PCLK2_Frequency;
    RCC_APB2PeriphClockCmd(instance->timClock, ENABLE);
  }

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  if(mode == TIM_PWM_INPUT_MODE)
    TIM_TimeBaseStructure.TIM_Period = 0xffff;
  else
    TIM_TimeBaseStructure.TIM_Period = instance->resolution/instance->hz -1;
  TIM_TimeBaseStructure.TIM_Prescaler = timerClocks/instance->resolution -1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(instance->timPort, &TIM_TimeBaseStructure);
}

void timPwmOutPortOpen(timPwmPort_t* instance, uint16_t timChan) {
  TIM_TypeDef* timx = instance->timPort;
  uint32_t gpioClock;
  uint16_t gpioPin;
  GPIO_TypeDef* gpioPort;

  switch(timChan) {
    case TIM_Channel_1:
      gpioClock = instance->chan1_gpioClock;
      gpioPin = instance->chan1_gpioPin;
      gpioPort = instance->chan1_gpioPort;
      break;
    case TIM_Channel_2:
      gpioClock = instance->chan2_gpioClock;
      gpioPin = instance->chan2_gpioPin;
      gpioPort = instance->chan2_gpioPort;
      break;
    case TIM_Channel_3:
      gpioClock = instance->chan3_gpioClock;
      gpioPin = instance->chan3_gpioPin;
      gpioPort = instance->chan3_gpioPort;
      break;
    case TIM_Channel_4:
      gpioClock = instance->chan4_gpioClock;
      gpioPin = instance->chan4_gpioPin;
      gpioPort = instance->chan4_gpioPort;
      break;
  }

  RCC_APB2PeriphClockCmd(gpioClock, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = gpioPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(gpioPort, &GPIO_InitStructure);

  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  /* only use Advanced Timer */
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OCIdleState_Set;

  switch(timChan) {
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

  /* only use Advenced Timer */
  if(instance->timClock & (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8))
    TIM_CtrlPWMOutputs(timx, ENABLE);

  TIM_Cmd(timx, ENABLE);
}

void timPwmInPortOpen(timPwmPort_t* instance, uint16_t timChan) {
  TIM_TypeDef* timx = instance->timPort;
  uint32_t gpioClock;
  uint16_t gpioPin;
  GPIO_TypeDef* gpioPort;

  switch(timChan) {
    case TIM_Channel_1:
      gpioClock = instance->chan1_gpioClock;
      gpioPin = instance->chan1_gpioPin;
      gpioPort = instance->chan1_gpioPort;
      break;
    case TIM_Channel_2:
      gpioClock = instance->chan2_gpioClock;
      gpioPin = instance->chan2_gpioPin;
      gpioPort = instance->chan2_gpioPort;
      break;
    case TIM_Channel_3:
      gpioClock = instance->chan3_gpioClock;
      gpioPin = instance->chan3_gpioPin;
      gpioPort = instance->chan3_gpioPort;
      break;
    case TIM_Channel_4:
      gpioClock = instance->chan4_gpioClock;
      gpioPin = instance->chan4_gpioPin;
      gpioPort = instance->chan4_gpioPort;
      break;
  }

  RCC_APB2PeriphClockCmd(gpioClock, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = gpioPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(gpioPort, &GPIO_InitStructure);

  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel = timChan;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(timx, &TIM_ICInitStructure);

  switch(timChan) {
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

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = instance->timIRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_Cmd(timx, ENABLE);
}

void timePwmOutWrite(timPwmPort_t* instance, uint16_t timChan, uint16_t duty) {
  TIM_TypeDef* timx = instance->timPort;

  duty %= (instance->resolution/instance->hz); // resolution/hz = (maxValue)

  switch(timChan) {
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

void timerPwmOpen(timPwmPort_t* instance, uint16_t timChan, timPwmMode_t mode) {
  timeBaseInit(instance, mode);
  switch(mode) {
    case TIM_PWM_INPUT_MODE:
      timPwmInPortOpen(instance, timChan);
      break;
    case TIM_PWM_OUTPUT_MODE:
      timPwmOutPortOpen(instance, timChan);
      break;
  }
}

/*
 * TIMER1
 */
void timerPwm1() {
  timPwm1.timClock = RCC_APB2Periph_TIM1;
  timPwm1.timPort = TIM1;
  timPwm1.timIRQ = TIM1_CC_IRQn;

  timPwm1.resolution = 1000000;
  timPwm1.hz = 250;
}

void timerPwm101_write(uint16_t duty) {timePwmOutWrite(&timPwm1, TIM_Channel_1, duty);}
uint16_t timerPwm101_read() {return timPwm1.duty[0];}
timPwmPort_t* openTimerPwm101(timPwmMode_t mode){
  timerPwm1();

  timPwm1.chan1_gpioPin = GPIO_Pin_8;
  timPwm1.chan1_gpioPort = GPIOA;
  timPwm1.chan1_gpioClock = RCC_APB2Periph_GPIOA;

  timPwm1.chan1Write = timerPwm101_write;
  timPwm1.chan1Read = timerPwm101_read;

  timerPwmOpen(&timPwm1, TIM_Channel_1, mode);

  return &timPwm1;
}

void timerPwm102_write(uint16_t duty) {timePwmOutWrite(&timPwm1, TIM_Channel_2, duty);}
uint16_t timerPwm102_read() {return timPwm1.duty[1];}
timPwmPort_t* openTimerPwm102(timPwmMode_t mode){
  timerPwm1();

  timPwm1.chan2_gpioPin = GPIO_Pin_9;
  timPwm1.chan2_gpioPort = GPIOA;
  timPwm1.chan2_gpioClock = RCC_APB2Periph_GPIOA;

  timPwm1.chan2Write = timerPwm102_write;
  timPwm1.chan2Read = timerPwm102_read;

  timerPwmOpen(&timPwm1, TIM_Channel_2, mode);

  return &timPwm1;
}

void timerPwm103_write(uint16_t duty) {timePwmOutWrite(&timPwm1, TIM_Channel_3, duty);}
uint16_t timerPwm103_read() {return timPwm1.duty[2];}
timPwmPort_t* openTimerPwm103(timPwmMode_t mode){
  timerPwm1();

  timPwm1.chan3_gpioPin = GPIO_Pin_10;
  timPwm1.chan3_gpioPort = GPIOA;
  timPwm1.chan3_gpioClock = RCC_APB2Periph_GPIOA;

  timPwm1.chan3Write = timerPwm103_write;
  timPwm1.chan3Read = timerPwm103_read;

  timerPwmOpen(&timPwm1, TIM_Channel_3, mode);

  return &timPwm1;
}

void timerPwm104_write(uint16_t duty) {timePwmOutWrite(&timPwm1, TIM_Channel_4, duty);}
uint16_t timerPwm104_read() {return timPwm1.duty[3];}
timPwmPort_t* openTimerPwm104(timPwmMode_t mode){
  timerPwm1();

  timPwm1.chan4_gpioPin = GPIO_Pin_11;
  timPwm1.chan4_gpioPort = GPIOA;
  timPwm1.chan4_gpioClock = RCC_APB2Periph_GPIOA;

  timPwm1.chan4Write = timerPwm104_write;
  timPwm1.chan4Read = timerPwm104_read;

  timerPwmOpen(&timPwm1, TIM_Channel_4, mode);

  return &timPwm1;
}

/*
 * TIMER2
 */
void timerPwm2() {
  timPwm2.timClock = RCC_APB1Periph_TIM2;
  timPwm2.timPort = TIM2;
  timPwm2.timIRQ = TIM2_IRQn;

  timPwm2.resolution = 1000000;
  timPwm2.hz = 250;
}

void timerPwm201_write(uint16_t duty) {timePwmOutWrite(&timPwm2, TIM_Channel_1, duty);}
uint16_t timerPwm201_read() {return timPwm2.duty[0];}
timPwmPort_t* openTimerPwm201(timPwmMode_t mode){
  timerPwm2();

  timPwm2.chan1_gpioPin = GPIO_Pin_0;
  timPwm2.chan1_gpioPort = GPIOA;
  timPwm2.chan1_gpioClock = RCC_APB2Periph_GPIOA;
  timPwm2.chan1Write = timerPwm201_write;
  timPwm2.chan1Read = timerPwm201_read;

  timerPwmOpen(&timPwm2, TIM_Channel_1, mode);

  return &timPwm2;
}
void timerPwm202_write(uint16_t duty) {timePwmOutWrite(&timPwm2, TIM_Channel_2, duty);}
uint16_t timerPwm202_read() {return timPwm2.duty[1];}
timPwmPort_t* openTimerPwm202(timPwmMode_t mode){
  timerPwm2();

  timPwm2.chan2_gpioPin = GPIO_Pin_1;
  timPwm2.chan2_gpioPort = GPIOA;
  timPwm2.chan2_gpioClock = RCC_APB2Periph_GPIOA;

  timPwm2.chan2Write = timerPwm202_write;
  timPwm2.chan2Read = timerPwm202_read;

  timerPwmOpen(&timPwm2, TIM_Channel_2, mode);

  return &timPwm2;
}

void timerPwm203_write(uint16_t duty) {timePwmOutWrite(&timPwm2, TIM_Channel_3, duty);}
uint16_t timerPwm203_read() {return timPwm2.duty[2];}
timPwmPort_t* openTimerPwm203(timPwmMode_t mode){
  timerPwm2();

  timPwm2.chan3_gpioPin = GPIO_Pin_2;
  timPwm2.chan3_gpioPort = GPIOA;
  timPwm2.chan3_gpioClock = RCC_APB2Periph_GPIOA;

  timPwm2.chan3Write = timerPwm203_write;
  timPwm2.chan3Read = timerPwm203_read;

  timerPwmOpen(&timPwm2, TIM_Channel_3, mode);

  return &timPwm2;
}

void timerPwm204_write(uint16_t duty) {timePwmOutWrite(&timPwm2, TIM_Channel_4, duty);}
uint16_t timerPwm204_read() {return timPwm2.duty[3];}
timPwmPort_t* openTimerPwm204(timPwmMode_t mode){
  timerPwm2();

  timPwm2.chan4_gpioPin = GPIO_Pin_3;
  timPwm2.chan4_gpioPort = GPIOA;
  timPwm2.chan4_gpioClock = RCC_APB2Periph_GPIOA;

  timPwm2.chan4Write = timerPwm204_write;
  timPwm2.chan4Read = timerPwm204_read;

  timerPwmOpen(&timPwm2, TIM_Channel_4, mode);

  return &timPwm2;
}

/*
 * TIMER3
 */
void timerPwm3() {
  timPwm3.timClock = RCC_APB1Periph_TIM3;
  timPwm3.timPort = TIM3;
  timPwm3.timIRQ = TIM3_IRQn;

  timPwm3.resolution = 1000000;
  timPwm3.hz = 250;
}

void timerPwm301_write(uint16_t duty) {timePwmOutWrite(&timPwm3, TIM_Channel_1, duty);}
uint16_t timerPwm301_read() {return timPwm3.duty[0];}
timPwmPort_t* openTimerPwm301(timPwmMode_t mode){
  timerPwm3();

  timPwm3.chan1_gpioPin = GPIO_Pin_6;
  timPwm3.chan1_gpioPort = GPIOA;
  timPwm3.chan1_gpioClock = RCC_APB2Periph_GPIOA;
  timPwm3.chan1Write = timerPwm301_write;
  timPwm3.chan1Read = timerPwm301_read;

  timerPwmOpen(&timPwm3, TIM_Channel_1, mode);

  return &timPwm3;
}

/*
 * error
 */
void timerPwm302_write(uint16_t duty) {timePwmOutWrite(&timPwm3, TIM_Channel_2, duty);}
uint16_t timerPwm302_read() {return timPwm3.duty[1];}
timPwmPort_t* openTimerPwm302(timPwmMode_t mode){
  timerPwm3();

  timPwm3.chan2_gpioPin = GPIO_Pin_7;
  timPwm3.chan2_gpioPort = GPIOA;
  timPwm3.chan2_gpioClock = RCC_APB2Periph_GPIOA;

  timPwm3.chan2Write = timerPwm302_write;
  timPwm3.chan2Read = timerPwm302_read;

  timerPwmOpen(&timPwm3, TIM_Channel_2, mode);

  return &timPwm3;
}

void timerPwm303_write(uint16_t duty) {timePwmOutWrite(&timPwm3, TIM_Channel_3, duty);}
uint16_t timerPwm303_read() {return timPwm3.duty[2];}
timPwmPort_t* openTimerPwm303(timPwmMode_t mode){
  timerPwm3();

  timPwm3.chan3_gpioPin = GPIO_Pin_0;
  timPwm3.chan3_gpioPort = GPIOB;
  timPwm3.chan3_gpioClock = RCC_APB2Periph_GPIOB;

  timPwm3.chan3Write = timerPwm303_write;
  timPwm3.chan3Read = timerPwm303_read;

  timerPwmOpen(&timPwm3, TIM_Channel_3, mode);

  return &timPwm3;
}

void timerPwm304_write(uint16_t duty) {timePwmOutWrite(&timPwm3, TIM_Channel_4, duty);}
uint16_t timerPwm304_read() {return timPwm3.duty[3];}
timPwmPort_t* openTimerPwm304(timPwmMode_t mode){
  timerPwm3();

  timPwm3.chan4_gpioPin = GPIO_Pin_1;
  timPwm3.chan4_gpioPort = GPIOB;
  timPwm3.chan4_gpioClock = RCC_APB2Periph_GPIOB;

  timPwm3.chan4Write = timerPwm304_write;
  timPwm3.chan4Read = timerPwm304_read;

  timerPwmOpen(&timPwm3, TIM_Channel_4, mode);

  return &timPwm3;
}

/*
 * TIMER4
 */
void timerPwm4() {
  timPwm4.timClock = RCC_APB1Periph_TIM4;
  timPwm4.timPort = TIM4;
  timPwm4.timIRQ = TIM4_IRQn;

  timPwm4.resolution = 1000000;
  timPwm4.hz = 250;
}

/*
 * ERROR
 */
void timerPwm401_write(uint16_t duty) {timePwmOutWrite(&timPwm4, TIM_Channel_1, duty);}
uint16_t timerPwm401_read() {return timPwm4.duty[0];}
timPwmPort_t* openTimerPwm401(timPwmMode_t mode){
  timerPwm3();

  timPwm4.chan1_gpioPin = GPIO_Pin_6;
  timPwm4.chan1_gpioPort = GPIOB;
  timPwm4.chan1_gpioClock = RCC_APB2Periph_GPIOB;
  timPwm4.chan1Write = timerPwm401_write;
  timPwm4.chan1Read = timerPwm401_read;

  timerPwmOpen(&timPwm4, TIM_Channel_1, mode);

  return &timPwm4;
}

/*
 * ERROR
 */
void timerPwm402_write(uint16_t duty) {timePwmOutWrite(&timPwm4, TIM_Channel_2, duty);}
uint16_t timerPwm402_read() {return timPwm4.duty[1];}
timPwmPort_t* openTimerPwm402(timPwmMode_t mode){
  timerPwm3();

  timPwm4.chan2_gpioPin = GPIO_Pin_7;
  timPwm4.chan2_gpioPort = GPIOB;
  timPwm4.chan2_gpioClock = RCC_APB2Periph_GPIOB;

  timPwm4.chan2Write = timerPwm402_write;
  timPwm4.chan2Read = timerPwm402_read;

  timerPwmOpen(&timPwm4, TIM_Channel_2, mode);

  return &timPwm4;
}

void timerPwm403_write(uint16_t duty) {timePwmOutWrite(&timPwm4, TIM_Channel_3, duty);}
uint16_t timerPwm403_read() {return timPwm4.duty[2];}
timPwmPort_t* openTimerPwm403(timPwmMode_t mode){
  timerPwm4();

  timPwm4.chan3_gpioPin = GPIO_Pin_8;
  timPwm4.chan3_gpioPort = GPIOB;
  timPwm4.chan3_gpioClock = RCC_APB2Periph_GPIOB;

  timPwm4.chan3Write = timerPwm403_write;
  timPwm4.chan3Read = timerPwm403_read;

  timerPwmOpen(&timPwm4, TIM_Channel_3, mode);

  return &timPwm4;
}

void timerPwm404_write(uint16_t duty) {timePwmOutWrite(&timPwm4, TIM_Channel_4, duty);}
uint16_t timerPwm404_read() {return timPwm4.duty[3];}
timPwmPort_t* openTimerPwm404(timPwmMode_t mode){
  timerPwm3();

  timPwm4.chan4_gpioPin = GPIO_Pin_9;
  timPwm4.chan4_gpioPort = GPIOB;
  timPwm4.chan4_gpioClock = RCC_APB2Periph_GPIOB;

  timPwm4.chan4Write = timerPwm404_write;
  timPwm4.chan4Read = timerPwm404_read;

  timerPwmOpen(&timPwm4, TIM_Channel_4, mode);

  return &timPwm4;
}

void timerHandler(timPwmPort_t* instance) {
  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_TypeDef* timx = instance->timPort;

  uint16_t capture, tim_ccer_ccnp;
  uint8_t channelNum;
  if(TIM_GetITStatus(timx, TIM_IT_CC1) != RESET) {
    TIM_ClearITPendingBit(timx, TIM_IT_CC1);
    capture = TIM_GetCapture1(timx);
    tim_ccer_ccnp = TIM_CCER_CC1P;
    channelNum = 1;
  }
  else if(TIM_GetITStatus(timx, TIM_IT_CC2) != RESET) {
    TIM_ClearITPendingBit(timx, TIM_IT_CC2);
    capture = TIM_GetCapture2(timx);
    tim_ccer_ccnp = TIM_CCER_CC2P;
    channelNum = 2;
  }
  else if(TIM_GetITStatus(timx, TIM_IT_CC3) != RESET) {
    TIM_ClearITPendingBit(timx, TIM_IT_CC3);
    capture = TIM_GetCapture3(timx);
    tim_ccer_ccnp = TIM_CCER_CC3P;
    channelNum = 3;
  }
  else if(TIM_GetITStatus(timx, TIM_IT_CC4) != RESET) {
    TIM_ClearITPendingBit(timx, TIM_IT_CC4);
    capture = TIM_GetCapture4(timx);
    tim_ccer_ccnp = TIM_CCER_CC4P;
    channelNum = 4;
  }

  if(instance->state[channelNum-1] == 0) {
    instance->riseTime[channelNum-1] = capture;
    instance->state[channelNum-1] = 1;
    timx->CCER |= tim_ccer_ccnp;
  }
  else {
    instance->fallTime[channelNum-1] = capture;
    instance->state[channelNum-1] = 0;
    instance->duty[channelNum-1] = instance->fallTime[channelNum-1] - instance->riseTime[channelNum-1];
    timx->CCER &= ~tim_ccer_ccnp;
  }
}

void TIM1_CC_IRQHandler() {
  timerHandler(&timPwm1);
}

void TIM2_IRQHandler() {
  timerHandler(&timPwm2);
}

void TIM3_IRQHandler() {
  timerHandler(&timPwm3);
}

void TIM4_IRQHandler() {
  timerHandler(&timPwm4);
}
