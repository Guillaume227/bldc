/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#pragma once

#include <stdint.h>

namespace buffer{
  void append_int16(uint8_t* buffer, int16_t number, int32_t *index);
  void append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
  void append_int32(uint8_t* buffer, int32_t number, int32_t *index);
  void append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
  void append_float16(uint8_t* buffer, float number, float scale, int32_t *index);
  void append_float32(uint8_t* buffer, float number, float scale, int32_t *index);
  void append_float32_auto(uint8_t* buffer, float number, int32_t *index);

  template<typename T>
  inline void append_float16(uint8_t* buffer, T number, float scale, int32_t *index){
    append_float16(buffer, static_cast<float>(number), scale, index);
  }
  template<typename T>
  inline void append_float32(uint8_t* buffer, T number, float scale, int32_t *index){
    append_float32(buffer, static_cast<float>(number), scale, index);
  }
  template<typename T>
  inline void append_float32_auto(uint8_t* buffer, T number, int32_t *index){
    append_float32_auto(buffer, static_cast<float>(number), index);
  }

  int16_t get_int16(const uint8_t *buffer, int32_t *index);
  uint16_t get_uint16(const uint8_t *buffer, int32_t *index);
  int32_t get_int32(const uint8_t *buffer, int32_t *index);
  uint32_t get_uint32(const uint8_t *buffer, int32_t *index);
  float get_float16(const uint8_t *buffer, float scale, int32_t *index);
  float get_float32(const uint8_t *buffer, float scale, int32_t *index);
  float get_float32_auto(const uint8_t *buffer, int32_t *index);

  template<typename T>
  inline void get_float16(T& out, const uint8_t *buffer, float scale, int32_t *index){
    out = T{get_float16(buffer, scale, index)};
  }
  template<typename T>
  inline void get_float32(T& out, const uint8_t *buffer, float scale, int32_t *index){
    out = T{get_float32(buffer, scale, index)};
  }
  template<typename T>
  inline void get_float32_auto(T& out, const uint8_t *buffer, int32_t *index){
    out = T{get_float32_auto(buffer, index)};
  }
}
