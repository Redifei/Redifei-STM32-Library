/*
 vcom.h - vcom library
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

#include "queue.h"

enum {
  RED_VCOM_PORT,
  RED_VCOM_PORT_MAX,
};

typedef enum {
  RED_VCOM_INTERRUPT_MODE,
} red_vcomMode_t;

typedef struct {
} red_vcom_hardware_t;

static const red_vcom_hardware_t redVcomWareMap[RED_VCOM_PORT_MAX] = {
};

typedef struct {
  red_vcomMode_t vcomMode;
} red_vcom_userSetting_t;

typedef struct {
  const red_vcom_hardware_t* hw;
  red_vcom_userSetting_t* userSetting;
  Qtype_t queue;
} red_vcom_setting_t;

typedef struct red_vcomPort {
  red_vcom_setting_t* setting;
  void (*putChar)(struct red_vcomPort* this, char);
  char (*getChar)(struct red_vcomPort* this);
  void (*printf)(struct red_vcomPort* this, char*, ...);
  bool (*available)(struct red_vcomPort* this);
} red_vcomPort_t;

#define IS_CONFIGED_VCOM_PORT(THIS_SETTING) (THIS_SETTING != NULL)
#define IS_VAILD_VCOM_PORT_NUM(VCOM_NUM) (VCOM_NUM < RED_VCOM_PORT_MAX)

void vcomQueuePush_in_hwConfig(char c); // This Function only use in hw_config.c
red_vcomPort_t* redVcomInit(uint8_t vcomPortNum, red_vcom_userSetting_t* userSetting);
