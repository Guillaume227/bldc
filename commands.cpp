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

#include "commands.h"
#include "ch.h"
#include "hal.h"
#include "mc_interface.h"
#include "stm32f4xx_conf.h"
#include "servo_simple.h"
#include "buffer.h"
#include "terminal.h"
#include "hw.h"
#include "mcpwm.h"
#include "mcpwm_foc.h"
#include "mc_interface.h"
#include "app.h"
#include "timeout.h"
#include "servo_dec.h"
#include "comm_can.h"
#include "flash_helper.h"
#include "utils.h"
#include "packet.h"
#include "encoder.h"
#include "nrf_driver.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

using namespace buffer;

namespace commands{

  // Threads
  THD_FUNCTION(detect_thread, arg);
  THD_WORKING_AREA(detect_thread_wa, 2048);
  thread_t *detect_tp;

  // Private variables
  uint8_t send_buffer[PACKET_MAX_PL_LEN];
  float detect_cycle_int_limit;
  float detect_coupling_k;
  float detect_current;
  float detect_min_rpm;
  float detect_low_duty;
  int8_t detect_hall_table[8];
  int detect_hall_res;
  void(*send_func)(unsigned char *data, unsigned int len) = 0;
  void(*send_func_last)(unsigned char *data, unsigned int len) = 0;
  void(*appdata_func)(unsigned char *data, unsigned int len) = 0;
  disp_pos_mode display_position_mode;

  void init(void) {
      chThdCreateStatic(detect_thread_wa, sizeof(detect_thread_wa), NORMALPRIO, detect_thread, NULL);
  }

  /**
   * Provide a function to use the next time there are packets to be sent.
   *
   * @param func
   * A pointer to the packet sending function.
   */
  void set_send_func(void(*func)(unsigned char *data, unsigned int len)) {
      send_func = func;
  }

  /**
   * Send a packet using the set send function.
   *
   * @param data
   * The packet data.
   *
   * @param len
   * The data length.
   */
  void send_packet(unsigned char *data, unsigned int len) {
      if (send_func) {
          send_func(data, len);
      }
  }

  /**
   * Process a received buffer with commands and data.
   *
   * @param data
   * The buffer to process.
   *
   * @param len
   * The length of the buffer.
   */
  void process_packet(unsigned char *data, unsigned int len) {
      if (!len) {
          return;
      }

      COMM_PACKET_ID packet_id;
      int32_t ind = 0;
      static mc_configuration mcconf, mcconf_old; // Static to save some stack space
      app_configuration appconf;
      uint16_t flash_res;
      uint32_t new_app_offset;
      chuck_data chuck_d_tmp;

      packet_id = data[0];
      data++;
      len--;

      switch (packet_id) {
      case COMM_FW_VERSION:
          ind = 0;
          send_buffer[ind++] = COMM_FW_VERSION;
          send_buffer[ind++] = FW_VERSION_MAJOR;
          send_buffer[ind++] = FW_VERSION_MINOR;

  #ifdef HW_NAME
          strcpy((char*)(send_buffer + ind), HW_NAME);
          ind += strlen(HW_NAME) + 1;

          memcpy(send_buffer + ind, STM32_UUID_8, 12);
          ind += 12;
  #endif

          send_packet(send_buffer, ind);
          break;

      case COMM_JUMP_TO_BOOTLOADER:
          flash_helper_jump_to_bootloader();
          break;

      case COMM_ERASE_NEW_APP:
          ind = 0;
          flash_res = flash_helper_erase_new_app(get_uint32(data, &ind));

          ind = 0;
          send_buffer[ind++] = COMM_ERASE_NEW_APP;
          send_buffer[ind++] = flash_res == FLASH_COMPLETE ? 1 : 0;
          send_packet(send_buffer, ind);
          break;

      case COMM_WRITE_NEW_APP_DATA:
          ind = 0;
          new_app_offset = get_uint32(data, &ind);
          flash_res = flash_helper_write_new_app_data(new_app_offset, data + ind, len - ind);

          ind = 0;
          send_buffer[ind++] = COMM_WRITE_NEW_APP_DATA;
          send_buffer[ind++] = flash_res == FLASH_COMPLETE ? 1 : 0;
          send_packet(send_buffer, ind);
          break;

      case COMM_GET_VALUES:
          ind = 0;
          send_buffer[ind++] = COMM_GET_VALUES;
          append_float16(send_buffer, mc_interface::temp_fet_filtered(), 1e1, &ind);
          append_float16(send_buffer, mc_interface::temp_motor_filtered(), 1e1, &ind);
          append_float32(send_buffer, mc_interface::read_reset_avg_motor_current(), 1e2, &ind);
          append_float32(send_buffer, mc_interface::read_reset_avg_input_current(), 1e2, &ind);
          append_float32(send_buffer, mc_interface::read_reset_avg_id(), 1e2, &ind);
          append_float32(send_buffer, mc_interface::read_reset_avg_iq(), 1e2, &ind);
          append_float16(send_buffer, mc_interface::get_duty_cycle_now(), 1e3, &ind);
          append_float32(send_buffer, mc_interface::get_rpm(), 1e0, &ind);
          append_float16(send_buffer, GET_INPUT_VOLTAGE(), 1e1, &ind);
          append_float32(send_buffer, mc_interface::get_amp_hours(false), 1e4, &ind);
          append_float32(send_buffer, mc_interface::get_amp_hours_charged(false), 1e4, &ind);
          append_float32(send_buffer, mc_interface::get_watt_hours(false), 1e4, &ind);
          append_float32(send_buffer, mc_interface::get_watt_hours_charged(false), 1e4, &ind);
          append_int32(send_buffer, mc_interface::get_tachometer_value(false), &ind);
          append_int32(send_buffer, mc_interface::get_tachometer_abs_value(false), &ind);
          send_buffer[ind++] = mc_interface::get_fault();
          append_float32(send_buffer, mc_interface::get_pid_pos_now(), 1e6, &ind);
          send_packet(send_buffer, ind);
          break;

      case COMM_SET_DUTY:
          ind = 0;
          mc_interface::set_duty((float)get_int32(data, &ind) / 100000.0);
          timeout::reset();
          break;

      case COMM_SET_CURRENT:
          ind = 0;
          mc_interface::set_current((float)get_int32(data, &ind) / 1000.0);
          timeout::reset();
          break;

      case COMM_SET_CURRENT_BRAKE:
          ind = 0;
          mc_interface::set_brake_current((float)get_int32(data, &ind) / 1000.0);
          timeout::reset();
          break;

      case COMM_SET_RPM:
          ind = 0;
          mc_interface::set_pid_speed((float)get_int32(data, &ind));
          timeout::reset();
          break;

      case COMM_SET_POS:
          ind = 0;
          mc_interface::set_pid_pos((float)get_int32(data, &ind) / 1000000.0);
          timeout::reset();
          break;

      case COMM_SET_HANDBRAKE:
          ind = 0;
          mc_interface::set_handbrake(get_float32(data, 1e3, &ind));
          timeout::reset();
          break;

      case COMM_SET_DETECT:
          mcconf = mc_interface::get_configuration();

          ind = 0;
          display_position_mode = static_cast<disp_pos_mode>(data[ind++]);

          if (mcconf.motor_type == MOTOR_TYPE_BLDC) {
              if (display_position_mode == DISP_POS_MODE_NONE) {
                  mc_interface::release_motor();
              } else if (display_position_mode == DISP_POS_MODE_INDUCTANCE) {
                  mcpwm::set_detect();
              }
          }

          timeout::reset();
          break;

      case COMM_SET_SERVO_POS:
  #if SERVO_OUT_ENABLE
          ind = 0;
          servo_simple_set_output(get_float16(data, 1000.0, &ind));
  #endif
          break;

      case COMM_SET_MCCONF:
          mcconf = mc_interface::get_configuration();

          ind = 0;
          mcconf.pwm_mode = data[ind++];
          mcconf.comm_mode = data[ind++];
          mcconf.motor_type = data[ind++];
          mcconf.sensor_mode = data[ind++];

          mcconf.l_current_max = get_float32_auto(data, &ind);
          mcconf.l_current_min = get_float32_auto(data, &ind);
          mcconf.l_in_current_max = get_float32_auto(data, &ind);
          mcconf.l_in_current_min = get_float32_auto(data, &ind);
          mcconf.l_abs_current_max = get_float32_auto(data, &ind);
          mcconf.l_min_erpm = get_float32_auto(data, &ind);
          mcconf.l_max_erpm = get_float32_auto(data, &ind);
          mcconf.l_erpm_start = get_float32_auto(data, &ind);
          mcconf.l_max_erpm_fbrake = get_float32_auto(data, &ind);
          mcconf.l_max_erpm_fbrake_cc = get_float32_auto(data, &ind);
          mcconf.l_min_vin = get_float32_auto(data, &ind);
          mcconf.l_max_vin = get_float32_auto(data, &ind);
          mcconf.l_battery_cut_start = get_float32_auto(data, &ind);
          mcconf.l_battery_cut_end = get_float32_auto(data, &ind);
          mcconf.l_slow_abs_current = data[ind++];
          mcconf.l_temp_fet_start = get_float32_auto(data, &ind);
          mcconf.l_temp_fet_end = get_float32_auto(data, &ind);
          mcconf.l_temp_motor_start = get_float32_auto(data, &ind);
          mcconf.l_temp_motor_end = get_float32_auto(data, &ind);
          mcconf.l_temp_accel_dec = get_float32_auto(data, &ind);
          mcconf.l_min_duty = get_float32_auto(data, &ind);
          mcconf.l_max_duty = get_float32_auto(data, &ind);
          mcconf.l_watt_max = get_float32_auto(data, &ind);
          mcconf.l_watt_min = get_float32_auto(data, &ind);

          mcconf.lo_current_max = mcconf.l_current_max;
          mcconf.lo_current_min = mcconf.l_current_min;
          mcconf.lo_in_current_max = mcconf.l_in_current_max;
          mcconf.lo_in_current_min = mcconf.l_in_current_min;
          mcconf.lo_current_motor_max_now = mcconf.l_current_max;
          mcconf.lo_current_motor_min_now = mcconf.l_current_min;

          mcconf.sl_min_erpm = get_float32_auto(data, &ind);
          mcconf.sl_min_erpm_cycle_int_limit = get_float32_auto(data, &ind);
          mcconf.sl_max_fullbreak_current_dir_change = get_float32_auto(data, &ind);
          mcconf.sl_cycle_int_limit = get_float32_auto(data, &ind);
          mcconf.sl_phase_advance_at_br = get_float32_auto(data, &ind);
          mcconf.sl_cycle_int_rpm_br = get_float32_auto(data, &ind);
          mcconf.sl_bemf_coupling_k = get_float32_auto(data, &ind);

          memcpy(mcconf.hall_table, data + ind, 8);
          ind += 8;
          mcconf.hall_sl_erpm = get_float32_auto(data, &ind);

          mcconf.foc_current_kp = get_float32_auto(data, &ind);
          mcconf.foc_current_ki = get_float32_auto(data, &ind);
          mcconf.foc_f_sw = get_float32_auto(data, &ind);
          mcconf.foc_dt_us = get_float32_auto(data, &ind);
          mcconf.foc_encoder_inverted = data[ind++];
          mcconf.foc_encoder_offset = get_float32_auto(data, &ind);
          mcconf.foc_encoder_ratio = get_float32_auto(data, &ind);
          mcconf.foc_sensor_mode = data[ind++];
          mcconf.foc_pll_kp = get_float32_auto(data, &ind);
          mcconf.foc_pll_ki = get_float32_auto(data, &ind);
          mcconf.foc_motor_l = get_float32_auto(data, &ind);
          mcconf.foc_motor_r = get_float32_auto(data, &ind);
          mcconf.foc_motor_flux_linkage = get_float32_auto(data, &ind);
          mcconf.foc_observer_gain = get_float32_auto(data, &ind);
          mcconf.foc_observer_gain_slow = get_float32_auto(data, &ind);
          mcconf.foc_duty_dowmramp_kp = get_float32_auto(data, &ind);
          mcconf.foc_duty_dowmramp_ki = get_float32_auto(data, &ind);
          mcconf.foc_openloop_rpm = get_float32_auto(data, &ind);
          mcconf.foc_sl_openloop_hyst = get_float32_auto(data, &ind);
          mcconf.foc_sl_openloop_time = get_float32_auto(data, &ind);
          mcconf.foc_sl_d_current_duty = get_float32_auto(data, &ind);
          mcconf.foc_sl_d_current_factor = get_float32_auto(data, &ind);
          memcpy(mcconf.foc_hall_table, data + ind, 8);
          ind += 8;
          mcconf.foc_sl_erpm = get_float32_auto(data, &ind);
          mcconf.foc_sample_v0_v7 = data[ind++];
          mcconf.foc_sample_high_current = data[ind++];
          mcconf.foc_sat_comp = get_float32_auto(data, &ind);
          mcconf.foc_temp_comp = data[ind++];
          mcconf.foc_temp_comp_base_temp = get_float32_auto(data, &ind);
          mcconf.foc_current_filter_const = get_float32_auto(data, &ind);

          mcconf.s_pid_kp = get_float32_auto(data, &ind);
          mcconf.s_pid_ki = get_float32_auto(data, &ind);
          mcconf.s_pid_kd = get_float32_auto(data, &ind);
          mcconf.s_pid_kd_filter = get_float32_auto(data, &ind);
          mcconf.s_pid_min_erpm = get_float32_auto(data, &ind);
          mcconf.s_pid_allow_braking = data[ind++];

          mcconf.p_pid_kp = get_float32_auto(data, &ind);
          mcconf.p_pid_ki = get_float32_auto(data, &ind);
          mcconf.p_pid_kd = get_float32_auto(data, &ind);
          mcconf.p_pid_kd_filter = get_float32_auto(data, &ind);
          mcconf.p_pid_ang_div = get_float32_auto(data, &ind);

          mcconf.cc_startup_boost_duty = get_float32_auto(data, &ind);
          mcconf.cc_min_current = get_float32_auto(data, &ind);
          mcconf.cc_gain = get_float32_auto(data, &ind);
          mcconf.cc_ramp_step_max = get_float32_auto(data, &ind);

          mcconf.m_fault_stop_time_ms = get_int32(data, &ind);
          mcconf.m_duty_ramp_step = get_float32_auto(data, &ind);
          mcconf.m_current_backoff_gain = get_float32_auto(data, &ind);
          mcconf.m_encoder_counts = get_uint32(data, &ind);
          mcconf.m_sensor_port_mode = data[ind++];
          mcconf.m_invert_direction = data[ind++];
          mcconf.m_drv8301_oc_mode = data[ind++];
          mcconf.m_drv8301_oc_adj = data[ind++];
          mcconf.m_bldc_f_sw_min = get_float32_auto(data, &ind);
          mcconf.m_bldc_f_sw_max = get_float32_auto(data, &ind);
          mcconf.m_dc_f_sw = get_float32_auto(data, &ind);
          mcconf.m_ntc_motor_beta = get_float32_auto(data, &ind);

          using utils::truncate_number;
          // Apply limits if they are defined
  #ifndef DISABLE_HW_LIMITS
  #ifdef HW_LIM_CURRENT
          truncate_number(mcconf.l_current_max, HW_LIM_CURRENT);
          truncate_number(mcconf.l_current_min, HW_LIM_CURRENT);
  #endif
  #ifdef HW_LIM_CURRENT_IN
          truncate_number(mcconf.l_in_current_max, HW_LIM_CURRENT_IN);
          truncate_number(mcconf.l_in_current_min, HW_LIM_CURRENT);
  #endif
  #ifdef HW_LIM_CURRENT_ABS
          truncate_number(mcconf.l_abs_current_max, HW_LIM_CURRENT_ABS);
  #endif
  #ifdef HW_LIM_VIN
          truncate_number(mcconf.l_max_vin, HW_LIM_VIN);
          truncate_number(mcconf.l_min_vin, HW_LIM_VIN);
  #endif
  #ifdef HW_LIM_ERPM
          truncate_number(mcconf.l_max_erpm, HW_LIM_ERPM);
          truncate_number(mcconf.l_min_erpm, HW_LIM_ERPM);
  #endif
  #ifdef HW_LIM_DUTY_MIN
          truncate_number(mcconf.l_min_duty, HW_LIM_DUTY_MIN);
  #endif
  #ifdef HW_LIM_DUTY_MAX
          truncate_number(mcconf.l_max_duty, HW_LIM_DUTY_MAX);
  #endif
  #ifdef HW_LIM_TEMP_FET
          truncate_number(mcconf.l_temp_fet_start, HW_LIM_TEMP_FET);
          truncate_number(mcconf.l_temp_fet_end, HW_LIM_TEMP_FET);
  #endif
  #endif

          conf_general::store_mc_configuration(&mcconf);
          mc_interface::set_configuration(&mcconf);
          chThdSleepMilliseconds(200);

          ind = 0;
          send_buffer[ind++] = packet_id;
          send_packet(send_buffer, ind);
          break;

      case COMM_GET_MCCONF:
      case COMM_GET_MCCONF_DEFAULT:
          if (packet_id == COMM_GET_MCCONF) {
              mcconf = mc_interface::get_configuration();
          } else {
              conf_general::get_default_mc_configuration(&mcconf);
          }

          ind = 0;
          send_buffer[ind++] = packet_id;

          send_buffer[ind++] = mcconf.pwm_mode;
          send_buffer[ind++] = mcconf.comm_mode;
          send_buffer[ind++] = mcconf.motor_type;
          send_buffer[ind++] = mcconf.sensor_mode;

          append_float32_auto(send_buffer, mcconf.l_current_max, &ind);
          append_float32_auto(send_buffer, mcconf.l_current_min, &ind);
          append_float32_auto(send_buffer, mcconf.l_in_current_max, &ind);
          append_float32_auto(send_buffer, mcconf.l_in_current_min, &ind);
          append_float32_auto(send_buffer, mcconf.l_abs_current_max, &ind);
          append_float32_auto(send_buffer, mcconf.l_min_erpm, &ind);
          append_float32_auto(send_buffer, mcconf.l_max_erpm, &ind);
          append_float32_auto(send_buffer, mcconf.l_erpm_start, &ind);
          append_float32_auto(send_buffer, mcconf.l_max_erpm_fbrake, &ind);
          append_float32_auto(send_buffer, mcconf.l_max_erpm_fbrake_cc, &ind);
          append_float32_auto(send_buffer, mcconf.l_min_vin, &ind);
          append_float32_auto(send_buffer, mcconf.l_max_vin, &ind);
          append_float32_auto(send_buffer, mcconf.l_battery_cut_start, &ind);
          append_float32_auto(send_buffer, mcconf.l_battery_cut_end, &ind);
          send_buffer[ind++] = mcconf.l_slow_abs_current;
          append_float32_auto(send_buffer, mcconf.l_temp_fet_start, &ind);
          append_float32_auto(send_buffer, mcconf.l_temp_fet_end, &ind);
          append_float32_auto(send_buffer, mcconf.l_temp_motor_start, &ind);
          append_float32_auto(send_buffer, mcconf.l_temp_motor_end, &ind);
          append_float32_auto(send_buffer, mcconf.l_temp_accel_dec, &ind);
          append_float32_auto(send_buffer, mcconf.l_min_duty, &ind);
          append_float32_auto(send_buffer, mcconf.l_max_duty, &ind);
          append_float32_auto(send_buffer, mcconf.l_watt_max, &ind);
          append_float32_auto(send_buffer, mcconf.l_watt_min, &ind);

          append_float32_auto(send_buffer, mcconf.sl_min_erpm, &ind);
          append_float32_auto(send_buffer, mcconf.sl_min_erpm_cycle_int_limit, &ind);
          append_float32_auto(send_buffer, mcconf.sl_max_fullbreak_current_dir_change, &ind);
          append_float32_auto(send_buffer, mcconf.sl_cycle_int_limit, &ind);
          append_float32_auto(send_buffer, mcconf.sl_phase_advance_at_br, &ind);
          append_float32_auto(send_buffer, mcconf.sl_cycle_int_rpm_br, &ind);
          append_float32_auto(send_buffer, mcconf.sl_bemf_coupling_k, &ind);

          memcpy(send_buffer + ind, mcconf.hall_table, 8);
          ind += 8;
          append_float32_auto(send_buffer, mcconf.hall_sl_erpm, &ind);

          append_float32_auto(send_buffer, mcconf.foc_current_kp, &ind);
          append_float32_auto(send_buffer, mcconf.foc_current_ki, &ind);
          append_float32_auto(send_buffer, mcconf.foc_f_sw, &ind);
          append_float32_auto(send_buffer, mcconf.foc_dt_us, &ind);
          send_buffer[ind++] = mcconf.foc_encoder_inverted;
          append_float32_auto(send_buffer, mcconf.foc_encoder_offset, &ind);
          append_float32_auto(send_buffer, mcconf.foc_encoder_ratio, &ind);
          send_buffer[ind++] = mcconf.foc_sensor_mode;
          append_float32_auto(send_buffer, mcconf.foc_pll_kp, &ind);
          append_float32_auto(send_buffer, mcconf.foc_pll_ki, &ind);
          append_float32_auto(send_buffer, mcconf.foc_motor_l, &ind);
          append_float32_auto(send_buffer, mcconf.foc_motor_r, &ind);
          append_float32_auto(send_buffer, mcconf.foc_motor_flux_linkage, &ind);
          append_float32_auto(send_buffer, mcconf.foc_observer_gain, &ind);
          append_float32_auto(send_buffer, mcconf.foc_observer_gain_slow, &ind);
          append_float32_auto(send_buffer, mcconf.foc_duty_dowmramp_kp, &ind);
          append_float32_auto(send_buffer, mcconf.foc_duty_dowmramp_ki, &ind);
          append_float32_auto(send_buffer, mcconf.foc_openloop_rpm, &ind);
          append_float32_auto(send_buffer, mcconf.foc_sl_openloop_hyst, &ind);
          append_float32_auto(send_buffer, mcconf.foc_sl_openloop_time, &ind);
          append_float32_auto(send_buffer, mcconf.foc_sl_d_current_duty, &ind);
          append_float32_auto(send_buffer, mcconf.foc_sl_d_current_factor, &ind);
          memcpy(send_buffer + ind, mcconf.foc_hall_table, 8);
          ind += 8;
          append_float32_auto(send_buffer, mcconf.foc_sl_erpm, &ind);
          send_buffer[ind++] = mcconf.foc_sample_v0_v7;
          send_buffer[ind++] = mcconf.foc_sample_high_current;
          append_float32_auto(send_buffer, mcconf.foc_sat_comp, &ind);
          send_buffer[ind++] = mcconf.foc_temp_comp;
          append_float32_auto(send_buffer, mcconf.foc_temp_comp_base_temp, &ind);
          append_float32_auto(send_buffer, mcconf.foc_current_filter_const, &ind);

          append_float32_auto(send_buffer, mcconf.s_pid_kp, &ind);
          append_float32_auto(send_buffer, mcconf.s_pid_ki, &ind);
          append_float32_auto(send_buffer, mcconf.s_pid_kd, &ind);
          append_float32_auto(send_buffer, mcconf.s_pid_kd_filter, &ind);
          append_float32_auto(send_buffer, mcconf.s_pid_min_erpm, &ind);
          send_buffer[ind++] = mcconf.s_pid_allow_braking;

          append_float32_auto(send_buffer, mcconf.p_pid_kp, &ind);
          append_float32_auto(send_buffer, mcconf.p_pid_ki, &ind);
          append_float32_auto(send_buffer, mcconf.p_pid_kd, &ind);
          append_float32_auto(send_buffer, mcconf.p_pid_kd_filter, &ind);
          append_float32_auto(send_buffer, mcconf.p_pid_ang_div, &ind);

          append_float32_auto(send_buffer, mcconf.cc_startup_boost_duty, &ind);
          append_float32_auto(send_buffer, mcconf.cc_min_current, &ind);
          append_float32_auto(send_buffer, mcconf.cc_gain, &ind);
          append_float32_auto(send_buffer, mcconf.cc_ramp_step_max, &ind);

          append_int32(send_buffer, mcconf.m_fault_stop_time_ms, &ind);
          append_float32_auto(send_buffer, mcconf.m_duty_ramp_step, &ind);
          append_float32_auto(send_buffer, mcconf.m_current_backoff_gain, &ind);
          append_uint32(send_buffer, mcconf.m_encoder_counts, &ind);
          send_buffer[ind++] = static_cast<uint8_t>(mcconf.m_sensor_port_mode);
          send_buffer[ind++] = mcconf.m_invert_direction;
          send_buffer[ind++] = static_cast<uint8_t>(mcconf.m_drv8301_oc_mode);
          send_buffer[ind++] = mcconf.m_drv8301_oc_adj;
          append_float32_auto(send_buffer, mcconf.m_bldc_f_sw_min, &ind);
          append_float32_auto(send_buffer, mcconf.m_bldc_f_sw_max, &ind);
          append_float32_auto(send_buffer, mcconf.m_dc_f_sw, &ind);
          append_float32_auto(send_buffer, mcconf.m_ntc_motor_beta, &ind);

          send_packet(send_buffer, ind);
          break;

      case COMM_SET_APPCONF:
          appconf = app::get_configuration();

          ind = 0;
          appconf.controller_id = data[ind++];
          appconf.timeout_msec = get_uint32(data, &ind);
          appconf.timeout_brake_current = get_float32_auto(data, &ind);
          appconf.send_can_status = data[ind++];
          appconf.send_can_status_rate_hz = get_uint16(data, &ind);
          appconf.can_baud_rate = static_cast<CAN_BAUD>(data[ind++]);

          appconf.app_to_use = data[ind++];

          appconf.app_ppm_conf.ctrl_type = data[ind++];
          appconf.app_ppm_conf.pid_max_erpm = get_float32_auto(data, &ind);
          appconf.app_ppm_conf.hyst = get_float32_auto(data, &ind);
          appconf.app_ppm_conf.pulse_start = get_float32_auto(data, &ind);
          appconf.app_ppm_conf.pulse_end = get_float32_auto(data, &ind);
          appconf.app_ppm_conf.pulse_center = get_float32_auto(data, &ind);
          appconf.app_ppm_conf.median_filter = data[ind++];
          appconf.app_ppm_conf.safe_start = data[ind++];
          appconf.app_ppm_conf.throttle_exp = get_float32_auto(data, &ind);
          appconf.app_ppm_conf.throttle_exp_brake = get_float32_auto(data, &ind);
          appconf.app_ppm_conf.throttle_exp_mode = data[ind++];
          appconf.app_ppm_conf.ramp_time_pos = get_float32_auto(data, &ind);
          appconf.app_ppm_conf.ramp_time_neg = get_float32_auto(data, &ind);
          appconf.app_ppm_conf.multi_esc = data[ind++];
          appconf.app_ppm_conf.tc = data[ind++];
          appconf.app_ppm_conf.tc_max_diff = get_float32_auto(data, &ind);

          appconf.app_adc_conf.ctrl_type = data[ind++];
          appconf.app_adc_conf.hyst = get_float32_auto(data, &ind);
          appconf.app_adc_conf.voltage_start = get_float32_auto(data, &ind);
          appconf.app_adc_conf.voltage_end = get_float32_auto(data, &ind);
          appconf.app_adc_conf.voltage_center = get_float32_auto(data, &ind);
          appconf.app_adc_conf.voltage2_start = get_float32_auto(data, &ind);
          appconf.app_adc_conf.voltage2_end = get_float32_auto(data, &ind);
          appconf.app_adc_conf.use_filter = data[ind++];
          appconf.app_adc_conf.safe_start = data[ind++];
          appconf.app_adc_conf.cc_button_inverted = data[ind++];
          appconf.app_adc_conf.rev_button_inverted = data[ind++];
          appconf.app_adc_conf.voltage_inverted = data[ind++];
          appconf.app_adc_conf.voltage2_inverted = data[ind++];
          appconf.app_adc_conf.throttle_exp = get_float32_auto(data, &ind);
          appconf.app_adc_conf.throttle_exp_brake = get_float32_auto(data, &ind);
          appconf.app_adc_conf.throttle_exp_mode = data[ind++];
          appconf.app_adc_conf.ramp_time_pos = get_float32_auto(data, &ind);
          appconf.app_adc_conf.ramp_time_neg = get_float32_auto(data, &ind);
          appconf.app_adc_conf.multi_esc = data[ind++];
          appconf.app_adc_conf.tc = data[ind++];
          appconf.app_adc_conf.tc_max_diff = get_float32_auto(data, &ind);
          appconf.app_adc_conf.update_rate_hz = get_uint16(data, &ind);

          appconf.app_uart_baudrate = get_uint32(data, &ind);

          appconf.app_chuk_conf.ctrl_type = data[ind++];
          appconf.app_chuk_conf.hyst = get_float32_auto(data, &ind);
          appconf.app_chuk_conf.ramp_time_pos = get_float32_auto(data, &ind);
          appconf.app_chuk_conf.ramp_time_neg = get_float32_auto(data, &ind);
          appconf.app_chuk_conf.stick_erpm_per_s_in_cc = get_float32_auto(data, &ind);
          appconf.app_chuk_conf.throttle_exp = get_float32_auto(data, &ind);
          appconf.app_chuk_conf.throttle_exp_brake = get_float32_auto(data, &ind);
          appconf.app_chuk_conf.throttle_exp_mode = data[ind++];
          appconf.app_chuk_conf.multi_esc = data[ind++];
          appconf.app_chuk_conf.tc = data[ind++];
          appconf.app_chuk_conf.tc_max_diff = get_float32_auto(data, &ind);

          appconf.app_nrf_conf.speed = data[ind++];
          appconf.app_nrf_conf.power = data[ind++];
          appconf.app_nrf_conf.crc_type = data[ind++];
          appconf.app_nrf_conf.retry_delay = data[ind++];
          appconf.app_nrf_conf.retries = data[ind++];
          appconf.app_nrf_conf.channel = data[ind++];
          memcpy(appconf.app_nrf_conf.address, data + ind, 3);
          ind += 3;
          appconf.app_nrf_conf.send_crc_ack = data[ind++];

          conf_general::store_app_configuration(&appconf);
          app::set_configuration(appconf);
          timeout::configure(appconf.timeout_msec, appconf.timeout_brake_current);
          chThdSleepMilliseconds(200);

          ind = 0;
          send_buffer[ind++] = packet_id;
          send_packet(send_buffer, ind);
          break;

      case COMM_GET_APPCONF:
      case COMM_GET_APPCONF_DEFAULT:
          if (packet_id == COMM_GET_APPCONF) {
              appconf = app::get_configuration();
          } else {
              conf_general::get_default_app_configuration(&appconf);
          }

          send_appconf(packet_id, &appconf);
          break;

      case COMM_SAMPLE_PRINT: {

          ind = 0;
          debug_sampling_mode mode = data[ind++];
          uint16_t sample_len = get_uint16(data, &ind);
          uint8_t decimation = data[ind++];
          mc_interface::sample_print_data(mode, sample_len, decimation);
      } break;

      case COMM_TERMINAL_CMD:
          data[len] = '\0';
          terminal::process_string((char*)data);
          break;

      case COMM_DETECT_MOTOR_PARAM:
          ind = 0;
          detect_current = get_float32(data, 1e3, &ind);
          detect_min_rpm = get_float32(data, 1e3, &ind);
          detect_low_duty = get_float32(data, 1e3, &ind);

          send_func_last = send_func;

          chEvtSignal(detect_tp, (eventmask_t) 1);
          break;

      case COMM_DETECT_MOTOR_R_L: {
          mcconf = mc_interface::get_configuration();
          mcconf_old = mcconf;

          send_func_last = send_func;

          mcconf.motor_type = MOTOR_TYPE_FOC;
          mc_interface::set_configuration(&mcconf);

          float r = 0.0;
          float l = 0.0;
          bool res = mcpwm_foc::measure_res_ind(r, l);
          mc_interface::set_configuration(&mcconf_old);

          if (!res) {
              r = 0.0;
              l = 0.0;
          }

          ind = 0;
          send_buffer[ind++] = COMM_DETECT_MOTOR_R_L;
          append_float32(send_buffer, r, 1e6, &ind);
          append_float32(send_buffer, l, 1e3, &ind);
          if (send_func_last) {
              send_func_last(send_buffer, ind);
          } else {
              send_packet(send_buffer, ind);
          }
      }
      break;

      case COMM_DETECT_MOTOR_FLUX_LINKAGE: {
          ind = 0;
          float current = get_float32(data, 1e3, &ind);
          float min_rpm = get_float32(data, 1e3, &ind);
          float duty = get_float32(data, 1e3, &ind);
          float resistance = get_float32(data, 1e6, &ind);

          send_func_last = send_func;

          float linkage;
          bool res = conf_general::measure_flux_linkage(current, duty, min_rpm, resistance, &linkage);

          if (!res) {
              linkage = 0.0;
          }

          ind = 0;
          send_buffer[ind++] = COMM_DETECT_MOTOR_FLUX_LINKAGE;
          append_float32(send_buffer, linkage, 1e7, &ind);
          if (send_func_last) {
              send_func_last(send_buffer, ind);
          } else {
              send_packet(send_buffer, ind);
          }
      }
      break;

      case COMM_DETECT_ENCODER: {
          if (encoder::is_configured()) {
              mcconf = mc_interface::get_configuration();
              mcconf_old = mcconf;

              send_func_last = send_func;

              ind = 0;
              float current = get_float32(data, 1e3, &ind);

              mcconf.motor_type = MOTOR_TYPE_FOC;
              mcconf.foc_f_sw = 10000.0;
              mcconf.foc_current_kp = 0.01;
              mcconf.foc_current_ki = 10.0;
              mc_interface::set_configuration(&mcconf);

              float offset = 0.0;
              float ratio = 0.0;
              bool inverted = false;
              mcpwm_foc::encoder_detect(current, false, offset, ratio, inverted);
              mc_interface::set_configuration(&mcconf_old);

              ind = 0;
              send_buffer[ind++] = COMM_DETECT_ENCODER;
              append_float32(send_buffer, offset, 1e6, &ind);
              append_float32(send_buffer, ratio, 1e6, &ind);
              send_buffer[ind++] = inverted;
              if (send_func_last) {
                  send_func_last(send_buffer, ind);
              } else {
                  send_packet(send_buffer, ind);
              }
          } else {
              ind = 0;
              send_buffer[ind++] = COMM_DETECT_ENCODER;
              append_float32(send_buffer, 1001.0, 1e6, &ind);
              append_float32(send_buffer, 0.0, 1e6, &ind);
              send_buffer[ind++] = false;
              send_packet(send_buffer, ind);
          }
      }
      break;

      case COMM_DETECT_HALL_FOC: {
          mcconf = mc_interface::get_configuration();

          if (mcconf.m_sensor_port_mode == SENSOR_PORT_MODE_HALL) {
              mcconf_old = mcconf;
              ind = 0;
              float current = get_float32(data, 1e3, &ind);

              send_func_last = send_func;

              mcconf.motor_type = MOTOR_TYPE_FOC;
              mcconf.foc_f_sw = 10000.0;
              mcconf.foc_current_kp = 0.01;
              mcconf.foc_current_ki = 10.0;
              mc_interface::set_configuration(&mcconf);

              uint8_t hall_tab[8];
              bool res = mcpwm_foc::hall_detect(current, hall_tab);
              mc_interface::set_configuration(&mcconf_old);

              ind = 0;
              send_buffer[ind++] = COMM_DETECT_HALL_FOC;
              memcpy(send_buffer + ind, hall_tab, 8);
              ind += 8;
              send_buffer[ind++] = res ? 0 : 1;

              if (send_func_last) {
                  send_func_last(send_buffer, ind);
              } else {
                  send_packet(send_buffer, ind);
              }
          } else {
              ind = 0;
              send_buffer[ind++] = COMM_DETECT_HALL_FOC;
              memset(send_buffer, 255, 8);
              ind += 8;
              send_buffer[ind++] = 0;
          }
      }
      break;

      case COMM_REBOOT:
          // Lock the system and enter an infinite loop. The watchdog will reboot.
          __disable_irq();
          for(;;){};
          break;

      case COMM_ALIVE:
          timeout::reset();
          break;

      case COMM_GET_DECODED_PPM:
          ind = 0;
          send_buffer[ind++] = COMM_GET_DECODED_PPM;
          append_int32(send_buffer, (int32_t)(app::ppm::get_decoded_level() * 1000000.0), &ind);
          append_int32(send_buffer, (int32_t)(servodec_get_last_pulse_len(0) * 1000000.0), &ind);
          send_packet(send_buffer, ind);
          break;

      case COMM_GET_DECODED_ADC:
          ind = 0;
          send_buffer[ind++] = COMM_GET_DECODED_ADC;
          append_int32(send_buffer, (int32_t)(app::adc::get_decoded_level() * 1000000.0), &ind);
          append_int32(send_buffer, (int32_t)(app::adc::get_voltage() * 1'000'000.0), &ind);
          append_int32(send_buffer, (int32_t)(app::adc::get_decoded_level2() * 1000000.0), &ind);
          append_int32(send_buffer, (int32_t)(app::adc::get_voltage2() * 1000000.0), &ind);
          send_packet(send_buffer, ind);
          break;

      case COMM_GET_DECODED_CHUK:
          ind = 0;
          send_buffer[ind++] = COMM_GET_DECODED_CHUK;
          append_int32(send_buffer, (int32_t)(app::nunchuk::get_decoded_chuk() * 1000000.0), &ind);
          send_packet(send_buffer, ind);
          break;

      case COMM_FORWARD_CAN:
          comm_can_send_buffer(data[0], data + 1, len - 1, false);
          break;

      case COMM_SET_CHUCK_DATA:
          ind = 0;
          chuck_d_tmp.js_x = data[ind++];
          chuck_d_tmp.js_y = data[ind++];
          chuck_d_tmp.bt_c = data[ind++];
          chuck_d_tmp.bt_z = data[ind++];
          chuck_d_tmp.acc_x = get_int16(data, &ind);
          chuck_d_tmp.acc_y = get_int16(data, &ind);
          chuck_d_tmp.acc_z = get_int16(data, &ind);
          app::nunchuk::update_output(&chuck_d_tmp);
          break;

      case COMM_CUSTOM_APP_DATA:
          if (appdata_func) {
              appdata_func(data, len);
          }
          break;

      case COMM_NRF_START_PAIRING:
          ind = 0;
          nrf_driver_start_pairing(get_int32(data, &ind));

          ind = 0;
          send_buffer[ind++] = packet_id;
          send_buffer[ind++] = NRF_PAIR_STARTED;
          send_packet(send_buffer, ind);
          break;

      default:
          break;
      }
  }

  void printf(const char* format, ...) {
      va_list arg;
      va_start (arg, format);
      int len;
      static char print_buffer[255];

      print_buffer[0] = COMM_PRINT;
      len = vsnprintf(print_buffer+1, 254, format, arg);
      va_end (arg);

      if(len > 0) {
          send_packet((unsigned char*)print_buffer, (len<254)? len+1: 255);
      }
  }

  void send_rotor_pos(float rotor_pos) {
      uint8_t buffer[5];
      int32_t index = 0;

      buffer[index++] = COMM_ROTOR_POSITION;
      append_int32(buffer, (int32_t)(rotor_pos * 100000.0), &index);

      send_packet(buffer, index);
  }

  void send_experiment_samples(float *samples, int len) {
      if ((len * 4 + 1) > 256) {
          return;
      }

      uint8_t buffer[len * 4 + 1];
      int32_t index = 0;

      buffer[index++] = COMM_EXPERIMENT_SAMPLE;

      for (int i = 0;i < len;i++) {
          append_int32(buffer, (int32_t)(samples[i] * 10000.0), &index);
      }

      send_packet(buffer, index);
  }

  disp_pos_mode get_disp_pos_mode(void) {
      return display_position_mode;
  }

  void set_app_data_handler(void(*func)(unsigned char *data, unsigned int len)) {
      appdata_func = func;
  }

  void send_app_data(unsigned char *data, unsigned int len) {
      int32_t index = 0;

      send_buffer[index++] = COMM_CUSTOM_APP_DATA;
      memcpy(send_buffer + index, data, len);
      index += len;

      send_packet(send_buffer, index);
  }

  void send_appconf(COMM_PACKET_ID packet_id, app_configuration *appconf) {
      int32_t ind = 0;
      send_buffer[ind++] = packet_id;
      send_buffer[ind++] = appconf->controller_id;
      append_uint32(send_buffer, appconf->timeout_msec, &ind);
      append_float32_auto(send_buffer, appconf->timeout_brake_current, &ind);
      send_buffer[ind++] = appconf->send_can_status;
      append_uint16(send_buffer, appconf->send_can_status_rate_hz, &ind);
      send_buffer[ind++] = static_cast<uint8_t>(appconf->can_baud_rate);

      send_buffer[ind++] = appconf->app_to_use;

      send_buffer[ind++] = appconf->app_ppm_conf.ctrl_type;
      append_float32_auto(send_buffer, appconf->app_ppm_conf.pid_max_erpm, &ind);
      append_float32_auto(send_buffer, appconf->app_ppm_conf.hyst, &ind);
      append_float32_auto(send_buffer, appconf->app_ppm_conf.pulse_start, &ind);
      append_float32_auto(send_buffer, appconf->app_ppm_conf.pulse_end, &ind);
      append_float32_auto(send_buffer, appconf->app_ppm_conf.pulse_center, &ind);
      send_buffer[ind++] = appconf->app_ppm_conf.median_filter;
      send_buffer[ind++] = appconf->app_ppm_conf.safe_start;
      append_float32_auto(send_buffer, appconf->app_ppm_conf.throttle_exp, &ind);
      append_float32_auto(send_buffer, appconf->app_ppm_conf.throttle_exp_brake, &ind);
      send_buffer[ind++] = appconf->app_ppm_conf.throttle_exp_mode;
      append_float32_auto(send_buffer, appconf->app_ppm_conf.ramp_time_pos, &ind);
      append_float32_auto(send_buffer, appconf->app_ppm_conf.ramp_time_neg, &ind);
      send_buffer[ind++] = appconf->app_ppm_conf.multi_esc;
      send_buffer[ind++] = appconf->app_ppm_conf.tc;
      append_float32_auto(send_buffer, appconf->app_ppm_conf.tc_max_diff, &ind);

      send_buffer[ind++] = appconf->app_adc_conf.ctrl_type;
      append_float32_auto(send_buffer, appconf->app_adc_conf.hyst, &ind);
      append_float32_auto(send_buffer, appconf->app_adc_conf.voltage_start, &ind);
      append_float32_auto(send_buffer, appconf->app_adc_conf.voltage_end, &ind);
      append_float32_auto(send_buffer, appconf->app_adc_conf.voltage_center, &ind);
      append_float32_auto(send_buffer, appconf->app_adc_conf.voltage2_start, &ind);
      append_float32_auto(send_buffer, appconf->app_adc_conf.voltage2_end, &ind);
      send_buffer[ind++] = appconf->app_adc_conf.use_filter;
      send_buffer[ind++] = appconf->app_adc_conf.safe_start;
      send_buffer[ind++] = appconf->app_adc_conf.cc_button_inverted;
      send_buffer[ind++] = appconf->app_adc_conf.rev_button_inverted;
      send_buffer[ind++] = appconf->app_adc_conf.voltage_inverted;
      send_buffer[ind++] = appconf->app_adc_conf.voltage2_inverted;
      append_float32_auto(send_buffer, appconf->app_adc_conf.throttle_exp, &ind);
      append_float32_auto(send_buffer, appconf->app_adc_conf.throttle_exp_brake, &ind);
      send_buffer[ind++] = appconf->app_adc_conf.throttle_exp_mode;
      append_float32_auto(send_buffer, appconf->app_adc_conf.ramp_time_pos, &ind);
      append_float32_auto(send_buffer, appconf->app_adc_conf.ramp_time_neg, &ind);
      send_buffer[ind++] = appconf->app_adc_conf.multi_esc;
      send_buffer[ind++] = appconf->app_adc_conf.tc;
      append_float32_auto(send_buffer, appconf->app_adc_conf.tc_max_diff, &ind);
      append_uint16(send_buffer, appconf->app_adc_conf.update_rate_hz, &ind);

      append_uint32(send_buffer, appconf->app_uart_baudrate, &ind);

      send_buffer[ind++] = appconf->app_chuk_conf.ctrl_type;
      append_float32_auto(send_buffer, appconf->app_chuk_conf.hyst, &ind);
      append_float32_auto(send_buffer, appconf->app_chuk_conf.ramp_time_pos, &ind);
      append_float32_auto(send_buffer, appconf->app_chuk_conf.ramp_time_neg, &ind);
      append_float32_auto(send_buffer, appconf->app_chuk_conf.stick_erpm_per_s_in_cc, &ind);
      append_float32_auto(send_buffer, appconf->app_chuk_conf.throttle_exp, &ind);
      append_float32_auto(send_buffer, appconf->app_chuk_conf.throttle_exp_brake, &ind);
      send_buffer[ind++] = appconf->app_chuk_conf.throttle_exp_mode;
      send_buffer[ind++] = appconf->app_chuk_conf.multi_esc;
      send_buffer[ind++] = appconf->app_chuk_conf.tc;
      append_float32_auto(send_buffer, appconf->app_chuk_conf.tc_max_diff, &ind);

      send_buffer[ind++] = appconf->app_nrf_conf.speed;
      send_buffer[ind++] = appconf->app_nrf_conf.power;
      send_buffer[ind++] = appconf->app_nrf_conf.crc_type;
      send_buffer[ind++] = appconf->app_nrf_conf.retry_delay;
      send_buffer[ind++] = appconf->app_nrf_conf.retries;
      send_buffer[ind++] = appconf->app_nrf_conf.channel;
      memcpy(send_buffer + ind, appconf->app_nrf_conf.address, 3);
      ind += 3;
      send_buffer[ind++] = appconf->app_nrf_conf.send_crc_ack;

      send_packet(send_buffer, ind);
  }

  THD_FUNCTION(detect_thread, arg) {
      (void)arg;

      chRegSetThreadName("Detect");

      detect_tp = chThdGetSelfX();

      for(;;) {
          chEvtWaitAny((eventmask_t) 1);

          if (!conf_general::detect_motor_param(detect_current, detect_min_rpm,
                  detect_low_duty, &detect_cycle_int_limit, &detect_coupling_k,
                  detect_hall_table, &detect_hall_res)) {
              detect_cycle_int_limit = 0.0;
              detect_coupling_k = 0.0;
          }

          int32_t ind = 0;
          send_buffer[ind++] = COMM_DETECT_MOTOR_PARAM;
          append_int32(send_buffer, (int32_t)(detect_cycle_int_limit * 1000.0), &ind);
          append_int32(send_buffer, (int32_t)(detect_coupling_k * 1000.0), &ind);
          memcpy(send_buffer + ind, detect_hall_table, 8);
          ind += 8;
          send_buffer[ind++] = detect_hall_res;

          if (send_func_last) {
              send_func_last(send_buffer, ind);
          } else {
              send_packet(send_buffer, ind);
          }
      }
  }
}
