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
  void set_duty(dutycycle_t dutyCycle);
  void set_duty_noramp(dutycycle_t dutyCycle);
  void set_pid_speed(rpm_t rpm);
  void set_pid_pos(degree_t pos);
  void set_current(ampere_t current);
  void set_brake_current(ampere_t current);
  void brake_now(void);
  void release_motor(void);
  int get_comm_step(void);
  dutycycle_t get_duty_cycle_set(void);
  dutycycle_t get_duty_cycle_now(void);
  hertz_t get_switching_frequency_now(void);
  rpm_t get_rpm(void);
  mc_state get_state(void);
  kv_t get_kv(void);
  kv_t get_kv_filtered(void);
  int get_tachometer_value(bool reset);
  int get_tachometer_abs_value(bool reset);
  void stop_pwm(void);
  ampere_t get_tot_current(void);
  ampere_t get_tot_current_filtered(void);
  ampere_t get_tot_current_directional(void);
  ampere_t get_tot_current_directional_filtered(void);
  ampere_t get_tot_current_in(void);
  ampere_t get_tot_current_in_filtered(void);
  void set_detect(void); // detect BLDC motor params
  degree_t get_detect_pos(void);
  void reset_hall_detect_table(void);
  int get_hall_detect_result(int8_t *table);
  int read_hall_phase(void);
  weber_t read_reset_avg_cycle_integrator(void);
  void set_comm_mode(mc_comm_mode mode);
  mc_comm_mode get_comm_mode(void);
  second_t get_last_adc_isr_duration(void);
  second_t get_last_inj_adc_isr_duration(void);
  mc_rpm_dep_struct const& get_rpm_dep(void);
  bool is_dccal_done(void);
  void switch_comm_mode(mc_comm_mode next);

  // Interrupt handlers
  void adc_interrupt_handler_injected(void);
  void adc_interrupt_handler(void *p, uint32_t flags);

  // External variables
  extern volatile float detect_currents[];
  extern volatile float detect_currents_diff[];
  extern volatile uint16_t detect_voltages[];
  extern volatile uint16_t mcpwm_vzero;

  /*
   * Fixed parameters
   */
  constexpr hertz_t RPM_TIMER_FREQ   = 1'000'000.0_Hz; // Frequency of the RPM measurement timer
  constexpr millisecond_t CMD_STOP_TIME      = 0_ms;  // Ignore commands for this duration in msec after a stop has been sent
  constexpr millisecond_t DETECT_STOP_TIME   = 500_ms;// Ignore commands for this duration in msec after a detect command

  // Speed PID parameters
  constexpr second_t PID_TIME_K       = 0.001_s;    // Pid controller sample time in seconds
}
