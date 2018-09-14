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
#include "hw.h"

namespace mc_interface{
// Functions
void init(mc_configuration const*configuration);
mc_configuration const& get_configuration(void);
void set_configuration(mc_configuration *configuration);
void set_pwm_callback(void (*p_func)(void));
void lock(void);
void unlock(void);
void lock_override_once(void);
mc_fault_code get_fault(void);
const char* fault_to_string(mc_fault_code fault);
mc_state get_state(void);
void set_duty(float dutyCycle);
void set_duty_noramp(float dutyCycle);
void set_pid_speed(float rpm);
void set_pid_pos(float pos);
void set_current(float current);
void set_brake_current(float current);
void set_current_rel(float val);
void set_brake_current_rel(float val);
void set_handbrake(float current);
void set_handbrake_rel(float val);
void brake_now(void);
void release_motor(void);
float get_duty_cycle_set(void);
float get_duty_cycle_now(void);
float get_sampling_frequency_now(void);
float get_rpm(void);
float get_amp_hours(bool reset);
float get_amp_hours_charged(bool reset);
float get_watt_hours(bool reset);
float get_watt_hours_charged(bool reset);
float get_tot_current(void);
float get_tot_current_filtered(void);
float get_tot_current_directional(void);
float get_tot_current_directional_filtered(void);
float get_tot_current_in(void);
float get_tot_current_in_filtered(void);
int get_tachometer_value(bool reset);
int get_tachometer_abs_value(bool reset);
float get_last_inj_adc_isr_duration(void); // in microseconds
float read_reset_avg_motor_current(void);
float read_reset_avg_input_current(void);
float read_reset_avg_id(void);
float read_reset_avg_iq(void);
float get_pid_pos_set(void);
float get_pid_pos_now(void);
float get_last_sample_adc_isr_duration(void);
void sample_print_data(debug_sampling_mode mode, uint16_t len, uint8_t decimation);
float temp_fet_filtered(void);
float temp_motor_filtered(void);

// MC implementation functions
void fault_stop(mc_fault_code fault);
int try_input(void);
void mc_timer_isr(void);

void set_duty_from_potentiometer(void);

// Interrupt handlers
void adc_inj_int_handler(void);
}
// External variables
extern volatile uint16_t ADC_Value[];
extern volatile int ADC_curr_norm_value[];

// Common fixed parameters
#ifndef HW_DEAD_TIME_VALUE
#define HW_DEAD_TIME_VALUE	60 // Dead time
#endif

#define TIM12_FREQ 10000000 // 10 MHz = 100ns
