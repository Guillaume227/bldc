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

namespace mcpwm {
  // Functions
  void init(mc_configuration *configuration);
  void deinit(void);
  bool init_done(void);
  void set_configuration(mc_configuration *configuration);
  void init_hall_table(int8_t *table);
  void set_duty(float dutyCycle);
  void set_duty_noramp(float dutyCycle);
  void set_pid_speed(float rpm);
  void set_pid_pos(float pos);
  void set_current(float current);
  void set_brake_current(float current);
  void brake_now(void);
  void release_motor(void);
  int get_comm_step(void);
  float get_duty_cycle_set(void);
  float get_duty_cycle_now(void);
  float get_switching_frequency_now(void);
  float get_rpm(void);
  mc_state get_state(void);
  float get_kv(void);
  float get_kv_filtered(void);
  int get_tachometer_value(bool reset);
  int get_tachometer_abs_value(bool reset);
  void stop_pwm(void);
  float get_tot_current(void);
  float get_tot_current_filtered(void);
  float get_tot_current_directional(void);
  float get_tot_current_directional_filtered(void);
  float get_tot_current_in(void);
  float get_tot_current_in_filtered(void);
  void set_detect(void); // detect BLDC motor params
  float get_detect_pos(void);
  void reset_hall_detect_table(void);
  int get_hall_detect_result(int8_t *table);
  int read_hall_phase(void);
  float read_reset_avg_cycle_integrator(void);
  void set_comm_mode(mc_comm_mode mode);
  mc_comm_mode get_comm_mode(void);
  float get_last_adc_isr_duration(void);
  float get_last_inj_adc_isr_duration(void);
  mc_rpm_dep_struct const& get_rpm_dep(void);
  bool is_dccal_done(void);
  void switch_comm_mode(mc_comm_mode next);

  // Interrupt handlers
  void adc_inj_int_handler(void);
  void adc_int_handler(void *p, uint32_t flags);

  // External variables
  extern volatile float detect_currents[];
  extern volatile float detect_voltages[];
  extern volatile float detect_currents_diff[];
  extern volatile int mcpwm_vzero;

  /*
   * Fixed parameters
   */
  constexpr float RPM_TIMER_FREQ   = 1000000.0;// Frequency of the RPM measurement timer
  constexpr int CMD_STOP_TIME      = 0;        // Ignore commands for this duration in msec after a stop has been sent
  constexpr int DETECT_STOP_TIME   = 500;      // Ignore commands for this duration in msec after a detect command

  // Speed PID parameters
  constexpr float PID_TIME_K       = 0.001;    // Pid controller sample time in seconds
}
