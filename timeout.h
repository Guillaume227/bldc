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

#include "ch.h"
#include "chtypes.h"
#include "chsystypes.h"

namespace timeout{

  // Functions
  void init(void);
  void configure(systime_t timeout, float brake_current);
  void reset(void);
  bool has_timeout(void);
  systime_t get_timeout_msec(void);
  float get_brake_current(void);

  class Disabler {
    systime_t const m_timeout_msec;
    float const m_brake_current;

  public:
    Disabler():
      m_timeout_msec(get_timeout_msec()),
      m_brake_current(get_brake_current())
    {
      // disable timeout
      reset();
      configure(60'000, 0.0);
    }

    ~Disabler(){
      // enable timeout
      configure(m_timeout_msec, m_brake_current);
    }
  };
}
