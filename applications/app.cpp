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

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "nrf_driver.h"
#include "rfhelp.h"
#include "comm_can.h"

namespace app {
  // Private variables
  app_configuration appconf;

  app_configuration const& get_configuration(void) {
      return appconf;
  }

  /**
   * Reconfigure and restart all apps. Some apps don't have any configuration options.
   *
   * @param conf
   * The new configuration to use.
   */
  void set_configuration(app_configuration const& conf) {
      appconf = conf;

      ppm::stop();
      adc::stop();
      uartcomm::stop();
      nunchuk::stop();

      if (!conf_general::permanent_nrf_found) {
          nrf_driver_stop();
      }

  #if CAN_ENABLE
      comm_can_set_baud(conf->can_baud_rate);
  #endif

  #ifdef APP_CUSTOM_TO_USE
      app_custom_stop();
  #endif

      switch (appconf.app_to_use) {
      case APP_PPM:
          ppm::start();
          break;

      case APP_ADC:
          adc::start(true);
          break;

      case APP_UART:
          hw_stop_i2c();
          uartcomm::start();
          break;

      case APP_PPM_UART:
          hw_stop_i2c();
          ppm::start();
          uartcomm::start();
          break;

      case APP_ADC_UART:
          hw_stop_i2c();
          adc::start(false);
          uartcomm::start();
          break;

      case APP_NUNCHUK:
          nunchuk::start();
          break;

      case APP_NRF:
          if (!conf_general::permanent_nrf_found) {
              nrf_driver_init();
              rfhelp_restart();
          }
          break;

      case APP_CUSTOM:
  #ifdef APP_CUSTOM_TO_USE
          hw_stop_i2c();
          custom_start();
  #endif
          break;

      default:
          break;
      }

      ppm::configure(&appconf.app_ppm_conf);
      adc::configure(&appconf.app_adc_conf);
      uartcomm::configure(appconf.app_uart_baudrate);
      nunchuk::configure(&appconf.app_chuk_conf);

  #ifdef APP_CUSTOM_TO_USE
      custom_configure(&appconf);
  #endif

      rfhelp_update_conf(&appconf.app_nrf_conf);
  }
}
