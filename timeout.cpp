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

#include "timeout.h"
#include "mc_interface.h"

namespace timeout{
  // Private variables
  volatile systime_t m_timeout_msec;
  volatile systime_t m_last_update_time;
  volatile float m_brake_current;
  volatile bool m_has_timeout;

  // Threads
  THD_WORKING_AREA(timeout_thread_wa, 512);
  THD_FUNCTION(timeout_thread, arg);

  void init(void) {
      m_timeout_msec = 1000;
      m_last_update_time = 0;
      m_brake_current = 0.0;
      m_has_timeout = false;

      chThdCreateStatic(timeout_thread_wa, sizeof(timeout_thread_wa), NORMALPRIO, timeout_thread, NULL);
  }

  void configure(systime_t timeout, float brake_current) {
      m_timeout_msec = timeout;
      m_brake_current = brake_current;
  }

  void reset(void) {
      m_last_update_time = chVTGetSystemTime();
  }

  bool has_timeout(void) {
      return m_has_timeout;
  }

  systime_t get_timeout_msec(void) {
      return m_timeout_msec;
  }

  float get_brake_current(void) {
      return m_brake_current;
  }

  THD_FUNCTION(timeout_thread, arg) {
      (void)arg;

      chRegSetThreadName("Timeout");

      for(;;) {
          if (m_timeout_msec != 0 && chVTTimeElapsedSinceX(m_last_update_time) > MS2ST(m_timeout_msec)) {
              mc_interface::unlock();
              mc_interface::set_brake_current(m_brake_current);
              m_has_timeout = true;
          } else {
              m_has_timeout = false;
          }

          chThdSleepMilliseconds(10);
      }
  }
}
