/*
	Copyright 2012-2017 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * hw.h
 *
 *  Created on: 12 apr 2014
 *      Author: benjamin
 */
#pragma once

#include "conf_general.h"
#include "stm32f4xx_conf.h"
#include "hal.h"

#ifdef HW_VERSION_40
#include "hw_40.h"
#elif defined HW_VERSION_45
#include "hw_45.h"
#elif defined HW_VERSION_46
#include "hw_46.h"
#elif defined HW_VERSION_48
#include "hw_48.h"
#elif defined HW_VERSION_49
#include "hw_49.h"
#elif defined HW_VERSION_410
#include "hw_410.h"
#elif defined HW_VERSION_60
#include "hw_60.h"
#elif defined HW_VERSION_R2
#include "hw_r2.h"
#elif defined HW_VERSION_VICTOR_R1A
#include "hw_victor_r1a.h"
#elif defined HW_VERSION_DAS_RS
#include "hw_das_rs.h"
#elif defined HW_VERSION_ARA
#include "hw_ara.h"
#elif defined HW_VERSION_PALTA
#include "hw_palta.h"
#elif defined HW_VERSION_RH
#include "hw_rh.h"
#elif defined HW_VERSION_TP
#include "hw_tp.h"
#elif defined HW_VERSION_75_300
#include "hw_75_300.h"
#elif defined HW_VERSION_MINI4
#include "hw_mini4.h"
#elif defined HW_VERSION_DAS_MINI
#include "hw_das_mini.h"
#else
#error "No hardware version defined"
#endif

namespace hw{

  /*
   * MCU
   */
  constexpr hertz_t SYSTEM_CORE_CLOCK{STM32_HCLK}; // TIM1, TIM8 clock //168'000'000 Hz, on F407, 180 MHz on F446
  constexpr hertz_t TIM2_CLOCK  = SYSTEM_CORE_CLOCK / 2; // assumes TIM2 clocks at half TIM1
  constexpr hertz_t TIM12_CLOCK = SYSTEM_CORE_CLOCK / 2; // assumes TIM12 clocks at half TIM1

  // Functions
  void init_gpio(void);
  void setup_adc_channels(void);
  void start_i2c(void);
  void stop_i2c(void);
  void try_restore_i2c(void);
}
