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

#pragma once

void systickTimerConfig();

uint32_t micros();
uint32_t millis();

void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);
