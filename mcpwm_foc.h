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

#ifndef MCPWM_FOC_H_
#define MCPWM_FOC_H_

#include "conf_general.h"
#include "datatypes.h"

namespace mcpwm_foc{
  // Functions
  void init(mc_configuration *configuration);
  void deinit(void);
  bool init_done(void);
  void set_configuration(mc_configuration *configuration);
  mc_state get_state(void);
  bool is_dccal_done(void);
  void stop_pwm(void);
  void set_duty(float dutyCycle);
  void set_duty_noramp(float dutyCycle);
  void set_pid_speed(float rpm);
  void set_pid_pos(float pos);
  void set_current(float current);
  void set_brake_current(float current);
  void set_handbrake(float current);
  void set_openloop(float current, float rpm);
  float get_duty_cycle_set(void);
  float get_duty_cycle_now(void);
  float get_pid_pos_set(void);
  float get_pid_pos_now(void);
  float get_switching_frequency_now(void);
  float get_sampling_frequency_now(void);
  float get_rpm(void);
  float get_tot_current(void);
  float get_tot_current_filtered(void);
  float get_abs_motor_current(void);
  float get_abs_motor_voltage(void);
  float get_abs_motor_current_filtered(void);
  float get_tot_current_directional(void);
  float get_tot_current_directional_filtered(void);
  float get_id(void);
  float get_iq(void);
  float get_tot_current_in(void);
  float get_tot_current_in_filtered(void);
  int get_tachometer_value(bool reset);
  int get_tachometer_abs_value(bool reset);
  float get_phase(void);
  float get_phase_observer(void);
  float get_phase_encoder(void);
  float get_vd(void);
  float get_vq(void);
  void encoder_detect(float current, bool print, float &offset, float &ratio, bool &inverted);
  float measure_resistance(float current, int samples);
  float measure_inductance(float duty, int samples, float *curr);
  bool measure_res_ind(float &res, float &ind);
  bool hall_detect(float current, uint8_t *hall_table);
  void print_state(void);
  float get_last_inj_adc_isr_duration(void);

  // Interrupt handlers
  void tim_sample_int_handler(void);
  void adc_int_handler(void *p, uint32_t flags);
}

// Defines
#define MCPWM_FOC_INDUCTANCE_SAMPLE_CNT_OFFSET		10 // Offset for the inductance measurement sample time in timer ticks
#define MCPWM_FOC_INDUCTANCE_SAMPLE_RISE_COMP		50 // Current rise time compensation
#define MCPWM_FOC_CURRENT_SAMP_OFFSET				(2) // Offset from timer top for injected ADC samples

#endif /* MCPWM_FOC_H_ */
