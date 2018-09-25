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

namespace packet{
  // Settings
  constexpr size_t RX_TIMEOUT		= 1000;
  constexpr size_t NUM_HANDLERS		= 2;
  constexpr size_t MAX_PL_LEN		= 1024;

  // Functions
  void init(void (*s_func)(unsigned char *data, unsigned int len),
            void (*p_func)(unsigned char *data, unsigned int len), int handler_num);
  void process_byte(uint8_t rx_data, int handler_num);
  void timerfunc(void);
  void send_packet(unsigned char *data, unsigned int len, int handler_num);
}
