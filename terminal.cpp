/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "ch.h"
#include "hal.h"
#include "terminal.h"
#include "mcpwm.h"
#include "mcpwm_foc.h"
#include "mc_interface.h"
#include "commands.h"
#include "hw.h"
#include "comm_can.h"
#include "utils.h"
#include "timeout.h"
#include "encoder.h"
#include "drv8301.h"
#include "drv8305.h"
#include "drv8320.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

// Settings
#define FAULT_VEC_LEN						25
#define CALLBACK_LEN						40

namespace {
  // Private types
  struct callback_struct {
      const char *command;
      const char *help;
      const char *arg_names;
      void(*cbf)(int argc, const char **argv);
  };

  // Private variables
  fault_data fault_vec[FAULT_VEC_LEN];
  volatile int fault_vec_write = 0;
  callback_struct callbacks[CALLBACK_LEN];
  int callback_write = 0;
}

namespace terminal{
  void process_string(char *str) {
      enum { kMaxArgs = 64 };
      int argc = 0;
      char *argv[kMaxArgs];

      char *p2 = strtok(str, " ");
      while (p2 && argc < kMaxArgs) {
          argv[argc++] = p2;
          p2 = strtok(0, " ");
      }
      using commands::printf;
      if (argc == 0) {
          printf("No command received\n");
          return;
      }

      static mc_configuration mcconf; // static to save some stack
      static mc_configuration mcconf_old; // static to save some stack
      mcconf = mc_interface::get_configuration();
      mcconf_old = mcconf;

      if (strcmp(argv[0], "ping") == 0) {
          printf("pong\n");
      } else if (strcmp(argv[0], "stop") == 0) {
          mc_interface::set_duty(0);
          printf("Motor stopped\n");
      } else if (strcmp(argv[0], "last_adc_duration") == 0) {
          printf("Latest ADC duration: %.4f ms", (double)(mcpwm::get_last_adc_isr_duration() * 1000.0));
          printf("Latest injected ADC duration: %.4f ms", (double)(mc_interface::get_last_inj_adc_isr_duration() * 1000.0));
          printf("Latest sample ADC duration: %.4f ms\n", (double)(mc_interface::get_last_sample_adc_isr_duration() * 1000.0));
      } else if (strcmp(argv[0], "kv") == 0) {
          printf("Calculated KV: %.2f rpm/volt\n", (double)mcpwm::get_kv_filtered());
      } else if (strcmp(argv[0], "mem") == 0) {
          size_t n, size;
          n = chHeapStatus(NULL, &size, NULL);
          printf("core free memory : %u bytes", chCoreGetStatusX());
          printf("heap fragments   : %u", n);
          printf("heap free total  : %u bytes\n", size);
      } else if (strcmp(argv[0], "threads") == 0) {
          thread_t *tp;
          static const char *states[] = {CH_STATE_NAMES};
          printf("    addr    stack prio refs     state           name time    ");
          printf("-------------------------------------------------------------");
          tp = chRegFirstThread();
          do {
              printf("%.8lx %.8lx %4lu %4lu %9s %14s %lu",
                      (uint32_t)tp, (uint32_t)tp->ctx.sp,
                      (uint32_t)tp->prio, (uint32_t)(tp->refs - 1),
                      states[tp->state], tp->name, (uint32_t)tp->time);
              tp = chRegNextThread(tp);
          } while (tp != NULL);
          printf("");
      } else if (strcmp(argv[0], "fault") == 0) {
          printf("%s\n", mc_interface::fault_to_string(mc_interface::get_fault()));
      } else if (strcmp(argv[0], "faults") == 0) {
          if (fault_vec_write == 0) {
              printf("No faults registered since startup\n");
          } else {
              printf("The following faults were registered since start:\n");
              for (int i = 0;i < fault_vec_write;i++) {
                  printf("Fault            : %s", mc_interface::fault_to_string(fault_vec[i].fault));
                  printf("Current          : %.1f", (double)fault_vec[i].current);
                  printf("Current filtered : %.1f", (double)fault_vec[i].current_filtered);
                  printf("Voltage          : %.2f", (double)fault_vec[i].voltage);
                  printf("Duty             : %.3f", (double)fault_vec[i].duty);
                  printf("RPM              : %.1f", (double)fault_vec[i].rpm);
                  printf("Tacho            : %d", fault_vec[i].tacho);
                  printf("Cycles running   : %d", fault_vec[i].cycles_running);
                  printf("TIM duty         : %d", (int)((float)fault_vec[i].tim_top * fault_vec[i].duty));
                  printf("TIM val samp     : %d", fault_vec[i].tim_val_samp);
                  printf("TIM current samp : %d", fault_vec[i].tim_current_samp);
                  printf("TIM top          : %d", fault_vec[i].tim_top);
                  printf("Comm step        : %d", fault_vec[i].comm_step);
                  printf("Temperature      : %.2f", (double)fault_vec[i].temperature);
  #ifdef HW_HAS_DRV8301
                  if (fault_vec[i].fault == FAULT_CODE_DRV) {
                      printf("DRV8301_FAULTS   : %s", drv8301_faults_to_string(fault_vec[i].drv8301_faults));
                  }
  #elif defined(HW_HAS_DRV8320)
                  if (fault_vec[i].fault == FAULT_CODE_DRV) {
                      printf("DRV8320_FAULTS   : %s", drv8320_faults_to_string(fault_vec[i].drv8301_faults));
                  }
  #endif
                  printf(" ");
              }
          }
      } else if (strcmp(argv[0], "rpm") == 0) {
          printf("Electrical RPM: %.2f rpm\n", (double)mc_interface::get_rpm());
      } else if (strcmp(argv[0], "tacho") == 0) {
          printf("Tachometer counts: %i\n", mc_interface::get_tachometer_value(0));
      } else if (strcmp(argv[0], "tim") == 0) {
          chSysLock();
          volatile int t1_cnt = TIM1->CNT;
          volatile int t8_cnt = TIM8->CNT;
          volatile int dir1 = !!(TIM1->CR1 & (1 << 4));
          volatile int dir8 = !!(TIM8->CR1 & (1 << 4));
          chSysUnlock();
          int duty1 = TIM1->CCR1;
          int duty2 = TIM1->CCR2;
          int duty3 = TIM1->CCR3;
          int top = TIM1->ARR;
          int voltage_samp = TIM8->CCR1;
          int current1_samp = TIM1->CCR4;
          int current2_samp = TIM8->CCR2;
          printf("Tim1 CNT: %i", t1_cnt);
          printf("Tim8 CNT: %u", t8_cnt);
          printf("Duty cycle1: %u", duty1);
          printf("Duty cycle2: %u", duty2);
          printf("Duty cycle3: %u", duty3);
          printf("Top: %u", top);
          printf("Dir1: %u", dir1);
          printf("Dir8: %u", dir8);
          printf("Voltage sample: %u", voltage_samp);
          printf("Current 1 sample: %u", current1_samp);
          printf("Current 2 sample: %u\n", current2_samp);
      } else if (strcmp(argv[0], "volt") == 0) {
          printf("Input voltage: %.2f\n", (double)GET_INPUT_VOLTAGE());
      } else if (strcmp(argv[0], "param_detect") == 0) {
          // Use COMM_MODE_DELAY and try to figure out the motor parameters.
          if (argc == 4) {
              float current = -1.0;
              float min_rpm = -1.0;
              float low_duty = -1.0;
              sscanf(argv[1], "%f", &current);
              sscanf(argv[2], "%f", &min_rpm);
              sscanf(argv[3], "%f", &low_duty);

              if (current > 0.0 && current < mcconf.l_current_max &&
                      min_rpm > 10.0 && min_rpm < 3000.0 &&
                      low_duty > 0.02 && low_duty < 0.8) {

                  float cycle_integrator;
                  float coupling_k;
                  int8_t hall_table[8];
                  int hall_res;
                  if (conf_general_detect_motor_param(current, min_rpm, low_duty, &cycle_integrator, &coupling_k, hall_table, &hall_res)) {
                      printf("Cycle integrator limit: %.2f", (double)cycle_integrator);
                      printf("Coupling factor: %.2f", (double)coupling_k);

                      if (hall_res == 0) {
                          printf("Detected hall sensor table:");
                          printf("%i, %i, %i, %i, %i, %i, %i, %i\n",
                                  hall_table[0], hall_table[1], hall_table[2], hall_table[3],
                                  hall_table[4], hall_table[5], hall_table[6], hall_table[7]);
                      } else if (hall_res == -1) {
                          printf("Hall sensor detection failed:");
                          printf("%i, %i, %i, %i, %i, %i, %i, %i\n",
                                  hall_table[0], hall_table[1], hall_table[2], hall_table[3],
                                  hall_table[4], hall_table[5], hall_table[6], hall_table[7]);
                      } else if (hall_res == -2) {
                          printf("WS2811 enabled. Hall sensors cannot be used.\n");
                      } else if (hall_res == -3) {
                          printf("Encoder enabled. Hall sensors cannot be used.\n");
                      }
                  } else {
                      printf("Detection failed. Try again with different parameters.\n");
                  }
              } else {
                  printf("Invalid argument(s).\n");
              }
          } else {
              printf("This command requires three arguments.\n");
          }
      } else if (strcmp(argv[0], "rpm_dep") == 0) {
          auto const& rpm_dep = mcpwm::get_rpm_dep();
          printf("Cycle int limit: %.2f", (double)rpm_dep.cycle_int_limit);
          printf("Cycle int limit running: %.2f", (double)rpm_dep.cycle_int_limit_running);
          printf("Cycle int limit max: %.2f\n", (double)rpm_dep.cycle_int_limit_max);
      } else if (strcmp(argv[0], "can_devs") == 0) {
          printf("CAN devices seen on the bus the past second:\n");
          for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
              can_status_msg *msg = comm_can_get_status_msg_index(i);

              if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 1.0) {
                  printf("ID                 : %i", msg->id);
                  printf("RX Time            : %i", msg->rx_time);
                  printf("Age (milliseconds) : %.2f", (double)(UTILS_AGE_S(msg->rx_time) * 1000.0));
                  printf("RPM                : %.2f", (double)msg->rpm);
                  printf("Current            : %.2f", (double)msg->current);
                  printf("Duty               : %.2f\n", (double)msg->duty);
              }
          }
      } else if (strcmp(argv[0], "foc_encoder_detect") == 0) {
          if (argc == 2) {
              float current = -1.0;
              sscanf(argv[1], "%f", &current);

              if (current > 0.0 && current <= mcconf.l_current_max) {
                  if (encoder::is_configured()) {
                      mc_motor_type type_old = mcconf.motor_type;
                      mcconf.motor_type = MOTOR_TYPE_FOC;
                      mc_interface::set_configuration(&mcconf);

                      float offset = 0.0;
                      float ratio = 0.0;
                      bool inverted = false;
                      mcpwm_foc::encoder_detect(current, true, &offset, &ratio, &inverted);

                      mcconf.motor_type = type_old;
                      mc_interface::set_configuration(&mcconf);

                      printf("Offset   : %.2f", (double)offset);
                      printf("Ratio    : %.2f", (double)ratio);
                      printf("Inverted : %s\n", inverted ? "true" : "false");
                  } else {
                      printf("Encoder not enabled.\n");
                  }
              } else {
                  printf("Invalid argument(s).\n");
              }
          } else {
              printf("This command requires one argument.\n");
          }
      } else if (strcmp(argv[0], "measure_res") == 0) {
          if (argc == 2) {
              float current = -1.0;
              sscanf(argv[1], "%f", &current);

              if (current > 0.0 && current <= mcconf.l_current_max) {
                  mcconf.motor_type = MOTOR_TYPE_FOC;
                  mc_interface::set_configuration(&mcconf);

                  printf("Resistance: %.6f ohm\n", (double)mcpwm_foc::measure_resistance(current, 2000));

                  mc_interface::set_configuration(&mcconf_old);
              } else {
                  printf("Invalid argument(s).\n");
              }
          } else {
              printf("This command requires one argument.\n");
          }
      } else if (strcmp(argv[0], "measure_ind") == 0) {
          if (argc == 2) {
              float duty = -1.0;
              sscanf(argv[1], "%f", &duty);

              if (duty > 0.0 && duty < 0.9) {
                  mcconf.motor_type = MOTOR_TYPE_FOC;
                  mcconf.foc_f_sw = 3000.0;
                  mc_interface::set_configuration(&mcconf);

                  float curr;
                  float ind = mcpwm_foc::measure_inductance(duty, 200, &curr);
                  printf("Inductance: %.2f microhenry (%.2f A)\n", (double)ind, (double)curr);

                  mc_interface::set_configuration(&mcconf_old);
              } else {
                  printf("Invalid argument(s).\n");
              }
          } else {
              printf("This command requires one argument.\n");
          }
      } else if (strcmp(argv[0], "measure_linkage") == 0) {
          if (argc == 5) {
              float current = -1.0;
              float duty = -1.0;
              float min_erpm = -1.0;
              float res = -1.0;
              sscanf(argv[1], "%f", &current);
              sscanf(argv[2], "%f", &duty);
              sscanf(argv[3], "%f", &min_erpm);
              sscanf(argv[4], "%f", &res);

              if (current > 0.0 && current <= mcconf.l_current_max && min_erpm > 0.0 && duty > 0.02 && res >= 0.0) {
                  float linkage;
                  conf_general_measure_flux_linkage(current, duty, min_erpm, res, &linkage);
                  printf("Flux linkage: %.7f\n", (double)linkage);
              } else {
                  printf("Invalid argument(s).\n");
              }
          } else {
              printf("This command requires one argument.\n");
          }
      } else if (strcmp(argv[0], "measure_res_ind") == 0) {
          mcconf.motor_type = MOTOR_TYPE_FOC;
          mc_interface::set_configuration(&mcconf);

          float res = 0.0;
          float ind = 0.0;
          mcpwm_foc::measure_res_ind(&res, &ind);
          printf("Resistance: %.6f ohm", (double)res);
          printf("Inductance: %.2f microhenry\n", (double)ind);

          mc_interface::set_configuration(&mcconf_old);
      } else if (strcmp(argv[0], "measure_linkage_foc") == 0) {
          if (argc == 2) {
              float duty = -1.0;
              sscanf(argv[1], "%f", &duty);

              if (duty > 0.0) {
                  mcconf.motor_type = MOTOR_TYPE_FOC;
                  mc_interface::set_configuration(&mcconf);
                  const float res = (3.0 / 2.0) * mcconf.foc_motor_r;

                  // Disable timeout
                  systime_t tout = timeout::get_timeout_msec();
                  float tout_c = timeout::get_brake_current();
                  timeout::reset();
                  timeout::configure(60000, 0.0);

                  for (int i = 0;i < 100;i++) {
                      mc_interface::set_duty(((float)i / 100.0) * duty);
                      chThdSleepMilliseconds(20);
                  }

                  float vq_avg = 0.0;
                  float rpm_avg = 0.0;
                  float samples = 0.0;
                  float iq_avg = 0.0;
                  for (int i = 0;i < 1000;i++) {
                      vq_avg += mcpwm_foc::get_vq();
                      rpm_avg += mc_interface::get_rpm();
                      iq_avg += mc_interface::get_tot_current_directional();
                      samples += 1.0;
                      chThdSleepMilliseconds(1);
                  }

                  mc_interface::release_motor();
                  mc_interface::set_configuration(&mcconf_old);

                  // Enable timeout
                  timeout::configure(tout, tout_c);

                  vq_avg /= samples;
                  rpm_avg /= samples;
                  iq_avg /= samples;

                  float linkage = (vq_avg - res * iq_avg) / (rpm_avg * ((2.0 * M_PI) / 60.0));

                  printf("Flux linkage: %.7f\n", (double)linkage);
              } else {
                  printf("Invalid argument(s).\n");
              }
          } else {
              printf("This command requires one argument.\n");
          }
      } else if (strcmp(argv[0], "foc_state") == 0) {
          mcpwm_foc::print_state();
          printf(" ");
      } else if (strcmp(argv[0], "hw_status") == 0) {
          printf("Firmware: %d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR);
  #ifdef HW_NAME
          printf("Hardware: %s", HW_NAME);
  #endif
          printf("UUID: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                  STM32_UUID_8[0], STM32_UUID_8[1], STM32_UUID_8[2], STM32_UUID_8[3],
                  STM32_UUID_8[4], STM32_UUID_8[5], STM32_UUID_8[6], STM32_UUID_8[7],
                  STM32_UUID_8[8], STM32_UUID_8[9], STM32_UUID_8[10], STM32_UUID_8[11]);
          printf("Permanent NRF found: %s", conf_general_permanent_nrf_found ? "Yes" : "No");
          printf(" ");
      } else if (strcmp(argv[0], "foc_openloop") == 0) {
          if (argc == 3) {
              float current = -1.0;
              float erpm = -1.0;
              sscanf(argv[1], "%f", &current);
              sscanf(argv[2], "%f", &erpm);

              if (current >= 0.0 && erpm >= 0.0) {
                  mcpwm_foc::set_openloop(current, erpm);
              } else {
                  printf("Invalid argument(s).\n");
              }
          } else {
              printf("This command requires two arguments.\n");
          }
      }

      // The help command
      else if (strcmp(argv[0], "help") == 0) {
          printf("Valid commands are:");
          printf("help");
          printf("  Show this help");

          printf("ping");
          printf("  Print pong here to see if the reply works");

          printf("stop");
          printf("  Stop the motor");

          printf("last_adc_duration");
          printf("  The time the latest ADC interrupt consumed");

          printf("kv");
          printf("  The calculated kv of the motor");

          printf("mem");
          printf("  Show memory usage");

          printf("threads");
          printf("  List all threads");

          printf("fault");
          printf("  Prints the current fault code");

          printf("faults");
          printf("  Prints all stored fault codes and conditions when they arrived");

          printf("rpm");
          printf("  Prints the current electrical RPM");

          printf("tacho");
          printf("  Prints tachometer value");

          printf("tim");
          printf("  Prints tim1 and tim8 settings");

          printf("volt");
          printf("  Prints different voltages");

          printf("param_detect [current] [min_rpm] [low_duty]");
          printf("  Spin up the motor in COMM_MODE_DELAY and compute its parameters.");
          printf("  This test should be performed without load on the motor.");
          printf("  Example: param_detect 5.0 600 0.06");

          printf("rpm_dep");
          printf("  Prints some rpm-dep values");

          printf("can_devs");
          printf("  Prints all CAN devices seen on the bus the past second");

          printf("foc_encoder_detect [current]");
          printf("  Run the motor at 1Hz on open loop and compute encoder settings");

          printf("measure_res [current]");
          printf("  Lock the motor with a current and calculate its resistance");

          printf("measure_ind [duty]");
          printf("  Send short voltage pulses, measure the current and calculate the motor inductance");

          printf("measure_linkage [current] [duty] [min_rpm] [motor_res]");
          printf("  Run the motor in BLDC delay mode and measure the flux linkage");
          printf("  example measure_linkage 5 0.5 700 0.076");
          printf("  tip: measure the resistance with measure_res first");

          printf("measure_res_ind");
          printf("  Measure the motor resistance and inductance with an incremental adaptive algorithm.");

          printf("measure_linkage_foc [duty]");
          printf("  Run the motor with FOC and measure the flux linkage.");

          printf("foc_state");
          printf("  Print some FOC state variables.");

          printf("hw_status");
          printf("  Print some hardware status information.");

          printf("foc_openloop [current] [erpm]");
          printf("  Create an open loop rotating current vector.");

          for (int i = 0;i < callback_write;i++) {
              if (callbacks[i].arg_names) {
                  printf("%s %s", callbacks[i].command, callbacks[i].arg_names);
              } else {
                  printf(callbacks[i].command);
              }

              if (callbacks[i].help) {
                  printf("  %s", callbacks[i].help);
              } else {
                  printf("  There is no help available for this command.");
              }
          }

          printf(" ");
      } else {
          bool found = false;
          for (int i = 0;i < callback_write;i++) {
              if (strcmp(argv[0], callbacks[i].command) == 0) {
                  callbacks[i].cbf(argc, (const char**)argv);
                  found = true;
                  break;
              }
          }

          if (!found) {
              printf("Invalid command: %s\n"
                      "type help to list all available commands\n", argv[0]);
          }
      }
  }

  void add_fault_data(fault_data *data) {
      fault_vec[fault_vec_write++] = *data;
      if (fault_vec_write >= FAULT_VEC_LEN) {
          fault_vec_write = 0;
      }
  }

  /**
   * Register a custom command  callback to the terminal. If the command
   * is already registered the old command callback will be replaced.
   *
   * @param command
   * The command name.
   *
   * @param help
   * A help text for the command. Can be NULL.
   *
   * @param arg_names
   * The argument names for the command, e.g. [arg_a] [arg_b]
   * Can be NULL.
   *
   * @param cbf
   * The callback function for the command.
   */
  void register_command_callback(
          const char* command,
          const char *help,
          const char *arg_names,
          void(*cbf)(int argc, const char **argv)) {

      int callback_num = callback_write;

      for (int i = 0;i < callback_write;i++) {
          // First check the address in case the same callback is registered more than once.
          if (callbacks[i].command == command) {
              callback_num = i;
              break;
          }

          // Check by string comparison.
          if (strcmp(callbacks[i].command, command) == 0) {
              callback_num = i;
              break;
          }
      }

      callbacks[callback_num].command = command;
      callbacks[callback_num].help = help;
      callbacks[callback_num].arg_names = arg_names;
      callbacks[callback_num].cbf = cbf;

      if (callback_num == callback_write) {
          callback_write++;
          if (callback_write >= CALLBACK_LEN) {
              callback_write = 0;
          }
      }
  }
}
