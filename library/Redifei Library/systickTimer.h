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

#pragma once

void systickTimerConfig();

uint32_t micros();
uint32_t millis();

void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);
