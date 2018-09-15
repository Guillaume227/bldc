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

#include "conf_general.h"

namespace encoder{
  // Functions
  void deinit(void);
  void init_abi(uint32_t counts);
  void init_as5047p_spi(void);
  bool is_configured(void);
  float read_deg(void);
  void reset(void);
  void tim_isr(void);
  void set_counts(uint32_t counts);
  bool index_found(void);
}
