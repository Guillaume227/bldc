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

#include <stdint.h>
#include "ch.h"
#include "units_def.h"

// Data types
enum mc_state {
   MC_STATE_OFF = 0,
   MC_STATE_DETECTING,
   MC_STATE_RUNNING,
   MC_STATE_FULL_BRAKE,
};

enum mc_pwm_mode {
	PWM_MODE_NONSYNCHRONOUS_HISW = 0, // This mode is not recommended
	PWM_MODE_SYNCHRONOUS, // The recommended and most tested mode
	PWM_MODE_BIPOLAR // Some glitches occasionally, can kill MOSFETs
};

enum mc_comm_mode {
	COMM_MODE_INTEGRATE = 0, // More robust, but requires many parameters.
	COMM_MODE_DELAY          // Like most hobby ESCs. Requires less parameters,
};

enum mc_sensor_mode {
	SENSOR_MODE_SENSORLESS = 0,
	SENSOR_MODE_SENSORED,
	SENSOR_MODE_HYBRID
};

enum mc_foc_sensor_mode {
	FOC_SENSOR_MODE_SENSORLESS = 0,
	FOC_SENSOR_MODE_ENCODER,
	FOC_SENSOR_MODE_HALL
};

enum mc_motor_type {
	MOTOR_TYPE_BLDC = 0,
	MOTOR_TYPE_DC,
	MOTOR_TYPE_FOC
};

enum mc_fault_code {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,
	FAULT_CODE_UNDER_VOLTAGE,
	FAULT_CODE_DRV,
	FAULT_CODE_ABS_OVER_CURRENT,
	FAULT_CODE_OVER_TEMP_FET,
	FAULT_CODE_OVER_TEMP_MOTOR
};

enum mc_control_mode {
	CONTROL_MODE_DUTY = 0,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_CURRENT,
	CONTROL_MODE_CURRENT_BRAKE,
	CONTROL_MODE_POS,
	CONTROL_MODE_HANDBRAKE,
	CONTROL_MODE_OPENLOOP,
	CONTROL_MODE_NONE
};

enum disp_pos_mode {
	DISP_POS_MODE_NONE = 0,
	DISP_POS_MODE_INDUCTANCE,
	DISP_POS_MODE_OBSERVER,
	DISP_POS_MODE_ENCODER,
	DISP_POS_MODE_PID_POS,
	DISP_POS_MODE_PID_POS_ERROR,
	DISP_POS_MODE_ENCODER_OBSERVER_ERROR
};

enum sensor_port_mode {
	SENSOR_PORT_MODE_HALL = 0,
	SENSOR_PORT_MODE_ABI,
	SENSOR_PORT_MODE_AS5047_SPI
};

struct mc_rpm_dep_struct {
	float cycle_int_limit;
	float cycle_int_limit_running;
	float cycle_int_limit_max;
	float comm_time_sum;
	float comm_time_sum_min_rpm;
	int32_t comms;
	uint32_t time_at_comm; // number of ticks spent at commutation step (6 step only)?
};

enum drv8301_oc_mode {
	DRV8301_OC_LIMIT = 0,
	DRV8301_OC_LATCH_SHUTDOWN,
	DRV8301_OC_REPORT_ONLY,
	DRV8301_OC_DISABLED
};

enum debug_sampling_mode {
	DEBUG_SAMPLING_OFF = 0,
	DEBUG_SAMPLING_NOW,
	DEBUG_SAMPLING_START,
	DEBUG_SAMPLING_TRIGGER_START,
	DEBUG_SAMPLING_TRIGGER_FAULT,
	DEBUG_SAMPLING_TRIGGER_START_NOSEND,
	DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND,
	DEBUG_SAMPLING_SEND_LAST_SAMPLES
};

enum CAN_BAUD {
	CAN_BAUD_125K = 0,
	CAN_BAUD_250K,
	CAN_BAUD_500K,
	CAN_BAUD_1M
};

struct mc_configuration {
	// Switching and drive
	mc_pwm_mode pwm_mode;
	mc_comm_mode comm_mode;
	mc_motor_type motor_type;
	mc_sensor_mode sensor_mode;
	// Limits
	ampere_t l_current_max;
	ampere_t l_current_min;
	ampere_t l_in_current_max;
	ampere_t l_in_current_min;
	ampere_t l_abs_current_max;
	rpm_t l_min_erpm;
	rpm_t l_max_erpm;
	scalar_t l_erpm_start;
	rpm_t l_max_erpm_fbrake;
	rpm_t l_max_erpm_fbrake_cc;
	volt_t l_min_vin;
	volt_t l_max_vin;
	volt_t l_battery_cut_start;
	volt_t l_battery_cut_end;
	bool l_slow_abs_current;
	celsius_t l_temp_fet_start;
	celsius_t l_temp_fet_end;
	celsius_t l_temp_motor_start;
	celsius_t l_temp_motor_end;
	celsius_t l_temp_accel_dec;
	dutycycle_t l_min_duty;
	dutycycle_t l_max_duty;
	watt_t l_watt_max;
	watt_t l_watt_min;
	// Overridden limits (Computed during runtime)
	ampere_t lo_current_max;
	ampere_t lo_current_min;
	ampere_t lo_in_current_max;
	ampere_t lo_in_current_min;
	ampere_t lo_current_motor_max_now;
	ampere_t lo_current_motor_min_now;
	// Sensorless (bldc)
	rpm_t sl_min_erpm;
	rpm_t sl_min_erpm_cycle_int_limit;
	ampere_t sl_max_fullbreak_current_dir_change;
	/*Cycle ingegrator limit. This is how much area will be integrated
	 * under the back EMF after a zero crossing before doing a commutation.
	 * A too low value will cause a too early commutation, and a too high value
	 * will cause a too late commutation.
	 * A too late commutation will cause more problems than too early commutations.
	 */
	float sl_cycle_int_limit;
	float sl_phase_advance_at_br;
	rpm_t sl_cycle_int_rpm_br;
	/*
     BEMF coupling. Roughly describes how much of the input voltage is seen on
     the BEMF at low modulation. Compensating for that at low speed helps the startup a lot.
	 parameter that defines the voltage coupling between the windings
	 when measuring the back-emf. This make a huge difference when running
	 at low speeds with low duty cycle.
	 This compensation has a RPM dependence though, which is something
	 I tried to avoid where possible because the RPM estimation has a delay
	 and thus causes problems during acceleration.
	 */
	float sl_bemf_coupling_k;
	// Hall sensor
	int8_t hall_table[8];
	rpm_t hall_sl_erpm;
	// FOC
	scalar_t foc_current_kp;
	hertz_t foc_current_ki;
	hertz_t foc_f_sw;
	microsecond_t foc_dt_us;
	degree_t foc_encoder_offset;
	bool foc_encoder_inverted;
	float foc_encoder_ratio;
	float foc_motor_l;
	float foc_motor_r;
	float foc_motor_flux_linkage;
	float foc_observer_gain;
	float foc_observer_gain_slow;
	hertz_t foc_pll_kp;
	inv_sec_2_t  foc_pll_ki;
	scalar_t foc_duty_dowmramp_kp;
	hertz_t foc_duty_dowmramp_ki;
	rpm_t foc_openloop_rpm;
	second_t foc_sl_openloop_hyst;
	second_t foc_sl_openloop_time;
	float foc_sl_d_current_duty;
	float foc_sl_d_current_factor;
	mc_foc_sensor_mode foc_sensor_mode;
	uint8_t foc_hall_table[8];
	rpm_t foc_sl_erpm;
	bool foc_sample_v0_v7;
	bool foc_sample_high_current;
	float foc_sat_comp;
	bool foc_temp_comp;
	float foc_temp_comp_base_temp;
	float foc_current_filter_const;
	// Speed PID
	scalar_t s_pid_kp;
	hertz_t s_pid_ki;
	second_t s_pid_kd;
	scalar_t s_pid_kd_filter;
	rpm_t s_pid_min_erpm;
	bool s_pid_allow_braking;
	// Pos PID
	scalar_t p_pid_kp;
	hertz_t p_pid_ki;
	second_t p_pid_kd;
	scalar_t p_pid_kd_filter;
	float p_pid_ang_div;
	// Current controller
	float cc_startup_boost_duty;
	ampere_t cc_min_current;
	float cc_gain;
	float cc_ramp_step_max;
	// Misc
	millisecond_t m_fault_stop_time_ms;
	float m_duty_ramp_step;
	float m_current_backoff_gain;
	uint32_t m_encoder_counts;
	sensor_port_mode m_sensor_port_mode;
	bool m_invert_direction;
	drv8301_oc_mode m_drv8301_oc_mode;
	int m_drv8301_oc_adj;
	hertz_t m_bldc_f_sw_min;
	hertz_t m_bldc_f_sw_max;
	hertz_t m_dc_f_sw;
	float m_ntc_motor_beta;
};

// Applications to use
enum app_use {
	APP_NONE = 0,
	APP_PPM,
	APP_ADC,
	APP_UART,
	APP_PPM_UART,
	APP_ADC_UART,
	APP_NUNCHUK,
	APP_NRF,
	APP_CUSTOM
};

// Throttle curve mode
enum thr_exp_mode {
	THR_EXP_EXPO = 0,
	THR_EXP_NATURAL,
	THR_EXP_POLY
};

// PPM control types
enum ppm_control_type {
	PPM_CTRL_TYPE_NONE = 0,
	PPM_CTRL_TYPE_CURRENT,
	PPM_CTRL_TYPE_CURRENT_NOREV,
	PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE,
	PPM_CTRL_TYPE_DUTY,
	PPM_CTRL_TYPE_DUTY_NOREV,
	PPM_CTRL_TYPE_PID,
	PPM_CTRL_TYPE_PID_NOREV
};

struct ppm_config {
	ppm_control_type ctrl_type;
	float pid_max_erpm;
	float hyst;
	float pulse_start;
	float pulse_end;
	float pulse_center;
	bool median_filter;
	bool safe_start;
	float throttle_exp;
	float throttle_exp_brake;
	thr_exp_mode throttle_exp_mode;
	float ramp_time_pos;
	float ramp_time_neg;
	bool multi_esc;
	bool tc;
	rpm_t tc_max_diff;
};

// ADC control types
enum adc_control_type {
	ADC_CTRL_TYPE_NONE = 0,
	ADC_CTRL_TYPE_CURRENT,
	ADC_CTRL_TYPE_CURRENT_REV_CENTER,
	ADC_CTRL_TYPE_CURRENT_REV_BUTTON,
	ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC,
	ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER,
	ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON,
	ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC,
	ADC_CTRL_TYPE_DUTY,
	ADC_CTRL_TYPE_DUTY_REV_CENTER,
	ADC_CTRL_TYPE_DUTY_REV_BUTTON,
	ADC_CTRL_TYPE_PID,
	ADC_CTRL_TYPE_PID_REV_CENTER,
	ADC_CTRL_TYPE_PID_REV_BUTTON
};

struct adc_config {
	adc_control_type ctrl_type;
	float hyst;
	volt_t voltage_start;
	volt_t voltage_end;
	volt_t voltage_center;
	volt_t voltage2_start;
	volt_t voltage2_end;
	bool use_filter;
	bool safe_start;
	bool cc_button_inverted;
	bool rev_button_inverted;
	bool voltage_inverted;
	bool voltage2_inverted;
	float throttle_exp;
	float throttle_exp_brake;
	thr_exp_mode throttle_exp_mode;
	float ramp_time_pos;
	float ramp_time_neg;
	bool multi_esc;
	bool tc;
	rpm_t tc_max_diff;
	uint32_t update_rate_hz;
};

// Nunchuk control types
enum chuk_control_type {
	CHUK_CTRL_TYPE_NONE = 0,
	CHUK_CTRL_TYPE_CURRENT,
	CHUK_CTRL_TYPE_CURRENT_NOREV
};

struct chuk_config {
	chuk_control_type ctrl_type;
	float hyst;
	float ramp_time_pos;
	float ramp_time_neg;
	rpm_t stick_erpm_per_s_in_cc;
	float throttle_exp;
	float throttle_exp_brake;
	thr_exp_mode throttle_exp_mode;
	bool multi_esc;
	bool tc;
	rpm_t tc_max_diff;
};

// NRF Datatypes
enum NRF_SPEED {
	NRF_SPEED_250K = 0,
	NRF_SPEED_1M,
	NRF_SPEED_2M
};

enum NRF_POWER {
	NRF_POWER_M18DBM = 0,
	NRF_POWER_M12DBM,
	NRF_POWER_M6DBM,
	NRF_POWER_0DBM,
  NRF_POWER_OFF
};

enum NRF_AW {
	NRF_AW_3 = 0,
	NRF_AW_4,
	NRF_AW_5
};

enum NRF_CRC {
	NRF_CRC_DISABLED = 0,
	NRF_CRC_1B,
	NRF_CRC_2B
};

enum NRF_RETR_DELAY {
	NRF_RETR_DELAY_250US = 0,
	NRF_RETR_DELAY_500US,
	NRF_RETR_DELAY_750US,
	NRF_RETR_DELAY_1000US,
	NRF_RETR_DELAY_1250US,
	NRF_RETR_DELAY_1500US,
	NRF_RETR_DELAY_1750US,
	NRF_RETR_DELAY_2000US,
	NRF_RETR_DELAY_2250US,
	NRF_RETR_DELAY_2500US,
	NRF_RETR_DELAY_2750US,
	NRF_RETR_DELAY_3000US,
	NRF_RETR_DELAY_3250US,
	NRF_RETR_DELAY_3500US,
	NRF_RETR_DELAY_3750US,
	NRF_RETR_DELAY_4000US
};

struct nrf_config {
	NRF_SPEED speed;
	NRF_POWER power;
	NRF_CRC crc_type;
	NRF_RETR_DELAY retry_delay;
	unsigned char retries;
	unsigned char channel;
	unsigned char address[3];
	bool send_crc_ack;
};

struct app_configuration {
	// Settings
	uint8_t controller_id;
	uint32_t timeout_msec;
	float timeout_brake_current;
	bool send_can_status;
	uint32_t send_can_status_rate_hz;
	CAN_BAUD can_baud_rate;

	// Application to use
	app_use app_to_use;

	// PPM application settings
	ppm_config app_ppm_conf;

	// ADC application settings
	adc_config app_adc_conf;

	// UART application settings
	uint32_t app_uart_baudrate;

	// Nunchuk application settings
	chuk_config app_chuk_conf;

	// NRF application settings
	nrf_config app_nrf_conf;
};

// Communication commands
enum COMM_PACKET_ID {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_HANDBRAKE,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_DETECT_HALL_FOC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA,
	COMM_NRF_START_PAIRING
};

// CAN commands
enum CAN_PACKET_ID {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL
};

// Logged fault data
struct fault_data {
	mc_fault_code fault;
	ampere_t current;
	ampere_t current_filtered;
	volt_t voltage;
	dutycycle_t duty;
	rpm_t rpm;
	int tacho;
	int cycles_running;
	int tim_val_samp;
	int tim_current_samp;
	int tim_top;
	int comm_step;
    celsius_t temperature;
	int drv8301_faults;
};

// External LED state
enum LED_EXT_STATE {
	LED_EXT_OFF = 0,
	LED_EXT_NORMAL,
	LED_EXT_BRAKE,
	LED_EXT_TURN_LEFT,
	LED_EXT_TURN_RIGHT,
	LED_EXT_BRAKE_TURN_LEFT,
	LED_EXT_BRAKE_TURN_RIGHT,
	LED_EXT_BATT
};

struct chuck_data {
	int js_x;
	int js_y;
	int acc_x;
	int acc_y;
	int acc_z;
	bool bt_c;
	bool bt_z;
};

struct can_status_msg {
	int id;
	systime_t rx_time;
	float rpm;
	float current;
	float duty;
};

struct mote_state {
	uint8_t js_x;
	uint8_t js_y;
	bool bt_c;
	bool bt_z;
	bool bt_push;
	float vbat;
};

enum MOTE_PACKET {
	MOTE_PACKET_BATT_LEVEL = 0,
	MOTE_PACKET_BUTTONS,
	MOTE_PACKET_ALIVE,
	MOTE_PACKET_FILL_RX_BUFFER,
	MOTE_PACKET_FILL_RX_BUFFER_LONG,
	MOTE_PACKET_PROCESS_RX_BUFFER,
	MOTE_PACKET_PROCESS_SHORT_BUFFER,
	MOTE_PACKET_PAIRING_INFO
};

struct mc_values {
	volt_t v_in;
	celsius_t temp_mos1;
	celsius_t temp_mos2;
	celsius_t temp_mos3;
	celsius_t temp_mos4;
	celsius_t temp_mos5;
	celsius_t temp_mos6;
	celsius_t temp_pcb;
	ampere_t current_motor;
    ampere_t current_in;
    float rpm;
    float duty_now;
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    int tachometer;
    int tachometer_abs;
    mc_fault_code fault_code;
};

enum NRF_PAIR_RES {
	NRF_PAIR_STARTED = 0,
	NRF_PAIR_OK,
	NRF_PAIR_FAIL
};
