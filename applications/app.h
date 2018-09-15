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

namespace app {
  // Functions
  app_configuration const& get_configuration(void);
  void set_configuration(app_configuration const& conf);

  // Standard apps
  namespace ppm {
    void start(void);
    void stop(void);
    float get_decoded_level(void);
    void configure(ppm_config *conf);
  }

  namespace adc {
    void start(bool use_rx_tx);
    void stop(void);
    void configure(adc_config *conf);
    float get_decoded_level(void);
    float get_voltage(void);
    float get_decoded_level2(void);
    float get_voltage2(void);
  }

  namespace uartcomm {
    void start(void);
    void stop(void);
    void configure(uint32_t baudrate);
  }

  namespace nunchuk{
    void start(void);
    void stop(void);
    void configure(chuk_config *conf);
    float get_decoded_chuk(void);
    void update_output(chuck_data *data);
  }

  // Custom apps
  namespace custom {
    void start(void);
    void stop(void);
    void configure(app_configuration *conf);
  }
}
