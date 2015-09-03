/*
 * Redifei: queue Library
 *
 * The code is released under the 'GNU GENERAL PUBLIC LICENSE Version 2'
 */

#pragma once

typedef struct {
  uint16_t head;
  uint16_t tail;
  uint16_t size;
  uint8_t* buf;
} Qtype_t;
