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

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mcpwm.h"
#include "mc_interface.h"
#include "digital_filter.h"
#include "utils.h"
#include "ledpwm.h"
#include "terminal.h"
#include "encoder.h"

using namespace utils;

namespace mcpwm {

// Global variables
  volatile float detect_currents[6];
  volatile float detect_voltages[6];
  volatile float detect_currents_diff[6];
  volatile int mcpwm_vzero;

  // Structs
  struct MCTimer {
    volatile bool updated;
    volatile unsigned int top;
    volatile unsigned int duty;
    volatile unsigned int val_sample;
    volatile unsigned int curr1_sample;
    volatile unsigned int curr2_sample;
#ifdef HW_HAS_3_SHUNTS
    volatile unsigned int curr3_sample;
#endif
  };

  // Private variables
  volatile int m_comm_step; // Range [1 6]
  volatile int m_detect_step; // Range [0 5]
  volatile int m_direction;
  volatile float m_dutycycle_set;
  volatile float m_dutycycle_now;
  volatile float m_rpm_now;
  volatile float m_speed_pid_set_rpm;
  volatile float m_pos_pid_set_pos;
  volatile float m_current_set;
  volatile int m_tachometer;
  volatile int m_tachometer_abs;
  volatile int m_tachometer_for_direction;
  volatile int m_curr0_sum;
  volatile int m_curr1_sum;
  volatile int m_curr_start_samples;
  volatile int m_curr0_offset;
  volatile int m_curr1_offset;
  volatile mc_state m_state;
  volatile mc_control_mode m_control_mode;
  volatile ampere_t m_last_current_sample;
  volatile ampere_t m_last_current_sample_filtered;
  volatile float m_detect_currents_avg[6];
  volatile float m_detect_avg_samples[6];

  /*To get better voltage samples, the switching frequency is adaptive and proportional
   * to the duty cycle. This is because the back-EMF only can be sampled
   * during the ON-time of the PWM cycle and low duty cycles have short ON time.
   * Since the motor is running slowly on low duty cycles, sampling and switching
   * does not have to be as fast to keep up with the motor
   * which makes this less of a problem.
   * Lower switching frequency also decreases switching losses,
   * which is a positive side-effect.
   */
  volatil_ hertz_t m_switching_frequency_now; // PWM switching frequency
  volatil_ millisecond_t m_ignore_iterations;
  MCTimer m_timer_struct;
  volatile int m_use_curr_samp_volt; // bitmap - Use the voltage-synchronized samples for this current sample
  int m_hall_to_phase_table[16];
  volatile unsigned int m_slow_ramping_cycles;
  volatile int m_has_commutated;
  mc_rpm_dep_struct rpm_dep;
  volatile float m_cycle_integrator_sum;
  volatile float m_cycle_integrator_iterations;
  mc_configuration *m_conf;
  volatil_ scalar_t m_pwm_cycles_sum;
  volatile int m_pwm_cycles;
  volatile float m_last_pwm_cycles_sum;
  volatile float m_last_pwm_cycles_sums[6];
  volatile bool m_dccal_done;
  volatile bool m_sensorless_now;
  volatile int m_hall_detect_table[8][7];
  volatile bool m_init_done;
  volatile mc_comm_mode m_comm_mode_next;

#ifdef HW_HAS_3_SHUNTS
  volatile int m_curr2_sum;
  volatile int m_curr2_offset;
#endif

  // KV FIR filter
  constexpr int KV_FIR_TAPS_BITS = 7;
  constexpr int KV_FIR_LEN = (1 << KV_FIR_TAPS_BITS);
  constexpr float KV_FIR_FCUT = 0.02;
  volatile float m_kv_fir_coeffs[KV_FIR_LEN];
  volatile float m_kv_fir_samples[KV_FIR_LEN];
  volatile uint32_t m_kv_fir_index = 0;

  // Amplitude FIR filter
  constexpr int AMP_FIR_TAPS_BITS = 7;
  constexpr int AMP_FIR_LEN = (1 << AMP_FIR_TAPS_BITS);
  constexpr float AMP_FIR_FCUT = 0.02;
  volatile float m_amp_fir_coeffs[AMP_FIR_LEN];
  volatile float m_amp_fir_samples[AMP_FIR_LEN];
  volatile uint32_t m_amp_fir_index = 0;

  // Current FIR filter
  constexpr int CURR_FIR_TAPS_BITS = 4;
  constexpr int CURR_FIR_LEN = (1 << CURR_FIR_TAPS_BITS);
  constexpr float CURR_FIR_FCUT = 0.15f;

  volatile float m_current_fir_coeffs[CURR_FIR_LEN];
  volatile float m_current_fir_samples[CURR_FIR_LEN];
  volatile uint32_t m_current_fir_index = 0;

  volatil_ second_t m_last_adc_isr_duration;
  volatil_ second_t m_last_inj_adc_isr_duration;

  // Private functions
  void set_duty_cycle_hl(float dutyCycle);
  void set_duty_cycle_ll(float dutyCycle);
  void set_duty_cycle_hw(float dutyCycle);
  void stop_pwm_ll(void);
  void stop_pwm_hw(void);
  void full_brake_ll(void);
  void full_brake_hw(void);
  void run_pid_control_speed(void);
  void run_pid_control_pos(second_t dt);
  void set_next_comm_step(int next_step);
  void update_rpm_tacho(void);
  void update_sensor_mode(void);
  int read_hall(void);
  void update_adc_sample_pos(MCTimer *timer_tmp);
  void commutate(int steps);
  void set_next_timer_settings(MCTimer const*settings);
  void update_timer_attempt(void);
  void set_switching_frequency(hertz_t frequency);
  void do_dc_cal(void);
  inline bool is_detecting() {
    return m_state == MC_STATE_DETECTING;
  }

  // Threads
  THD_WORKING_AREA(timer_thread_wa, 2048);
  THD_FUNCTION(timer_thread, arg);
  THD_WORKING_AREA(rpm_thread_wa, 1024);
  THD_FUNCTION(rpm_thread, arg);
  volatile bool timer_thd_stop;
  volatile bool rpm_thd_stop;

  void init(mc_configuration *configuration) {
    sys_lock_cnt();

    m_init_done = false;

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    m_conf = configuration;

    // Initialize variables
    m_comm_step = 1;
    m_detect_step = 0;
    m_direction = 1;
    m_rpm_now = 0.0;
    m_dutycycle_set = 0.0;
    m_dutycycle_now = 0.0;
    m_speed_pid_set_rpm = 0.0;
    m_pos_pid_set_pos = 0.0;
    m_current_set = 0.0;
    m_tachometer = 0;
    m_tachometer_abs = 0;
    m_tachometer_for_direction = 0;
    m_state = MC_STATE_OFF;
    m_control_mode = CONTROL_MODE_NONE;
    m_last_current_sample = 0.0;
    m_last_current_sample_filtered = 0.0;
    m_switching_frequency_now = m_conf->m_bldc_f_sw_max;
    m_ignore_iterations = 0_ms;
    m_use_curr_samp_volt = 0;
    m_slow_ramping_cycles = 0;
    m_has_commutated = 0;
    memset((void*)&rpm_dep, 0, sizeof(rpm_dep));
    m_cycle_integrator_sum = 0.0;
    m_cycle_integrator_iterations = 0.0;
    m_pwm_cycles_sum = 0.0;
    m_pwm_cycles = 0;
    m_last_pwm_cycles_sum = 0.0;
    memset((float*)m_last_pwm_cycles_sums, 0, sizeof(m_last_pwm_cycles_sums));
    m_dccal_done = false;
    memset((void*)m_hall_detect_table, 0,
           sizeof(m_hall_detect_table[0][0]) * 8 * 7);
    update_sensor_mode();
    m_comm_mode_next = m_conf->comm_mode;

    init_hall_table((int8_t*)m_conf->hall_table);

    // Create KV FIR filter
    filter::create_fir_lowpass((float*)m_kv_fir_coeffs, KV_FIR_FCUT,
                               KV_FIR_TAPS_BITS, 1);

    // Create amplitude FIR filter
    filter::create_fir_lowpass((float*)m_amp_fir_coeffs, AMP_FIR_FCUT,
                               AMP_FIR_TAPS_BITS, 1);

    // Create current FIR filter
    filter::create_fir_lowpass((float*)m_current_fir_coeffs, CURR_FIR_FCUT,
                               CURR_FIR_TAPS_BITS, 1);

    TIM_DeInit(TIM1);
    TIM_DeInit(TIM8);
    TIM1->CNT = 0;
    TIM8->CNT = 0;

    // TIM1 clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // Time Base configuration
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK
        / (int)static_cast<float>(m_switching_frequency_now); // ARR value
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // Channel 1, 2 and 3 Configuration in PWM mode
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2; // Pulse (CCRx): the duty cycle - ARR/2 = 50% duty	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // Automatic Output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = HW_DEAD_TIME_VALUE;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
    TIM_CCPreloadControl(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    /*
     * ADC!
     */
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // Clock
    RCC_AHB1PeriphClockCmd(
        RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC,
        ENABLE);
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3,
        ENABLE);

    dmaStreamAllocate(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)), 3,
                      (stm32_dmaisr_t)adc_int_handler, (void *)0);

    // DMA for the ADC
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) & ADC_Value;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & ADC->CDR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);

    // DMA2_Stream0 enable
    DMA_Cmd(DMA2_Stream4, ENABLE);

    // Enable transfer complete interrupt
    DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);

    // ADC Common Init
    // Note that the ADC is running at 45MHz (= 90MHz (APHB2 freq on F446) / Div2 (ADC prescaler))
    // which is higher than the specified 36MHz in the data sheet, but it works.
    ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // Channel-specific settings
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge =
        ADC_ExternalTrigConvEdge_Falling;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;

    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = 0;
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_Init(ADC3, &ADC_InitStructure);

    // Enable Vrefint channel
    ADC_TempSensorVrefintCmd(ENABLE);

    // Enable DMA request after last transfer (Multi-ADC mode)
    ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

    // Injected channels for current measurement at end of cycle
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T8_CC2);
#ifdef HW_HAS_3_SHUNTS
    ADC_ExternalTrigInjectedConvConfig(ADC3, ADC_ExternalTrigInjecConv_T8_CC3);
#endif
    ADC_ExternalTrigInjectedConvEdgeConfig(
        ADC1, ADC_ExternalTrigInjecConvEdge_Falling);
    ADC_ExternalTrigInjectedConvEdgeConfig(
        ADC2, ADC_ExternalTrigInjecConvEdge_Falling);
#ifdef HW_HAS_3_SHUNTS
    ADC_ExternalTrigInjectedConvEdgeConfig(
        ADC3, ADC_ExternalTrigInjecConvEdge_Falling);
#endif
    ADC_InjectedSequencerLengthConfig(ADC1, HW_ADC_INJ_CHANNELS);
    ADC_InjectedSequencerLengthConfig(ADC2, HW_ADC_INJ_CHANNELS);
#ifdef HW_HAS_3_SHUNTS
    ADC_InjectedSequencerLengthConfig(ADC3, HW_ADC_INJ_CHANNELS);
#endif

    hw_setup_adc_channels();

    // Interrupt
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    nvicEnableVector(ADC_IRQn, 6);

    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);

    // Enable ADC2
    ADC_Cmd(ADC2, ENABLE);

    // Enable ADC3
    ADC_Cmd(ADC3, ENABLE);

    // ------------- Timer8 for ADC sampling ------------- //
    // Time Base configuration
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 500;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_CCPreloadControl(TIM8, ENABLE);

    // PWM outputs have to be enabled in order to trigger ADC on CCx
    TIM_CtrlPWMOutputs(TIM8, ENABLE);

    // TIM1 Master and TIM8 slave
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
    TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
    TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);

    // Enable TIM1 and TIM8
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM8, ENABLE);

    // Main Output Enable
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // 32-bit timer for RPM measurement
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    uint16_t PrescalerValue = (uint16_t)static_cast<float>(TIM2_CLOCK / RPM_TIMER_FREQ) - 1;

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // TIM2 enable counter
    TIM_Cmd(TIM2, ENABLE);

    // ADC sampling locations
    stop_pwm_hw();
    MCTimer timer_tmp;
    timer_tmp.top = TIM1->ARR;
    timer_tmp.duty = TIM1->ARR / 2;
    update_adc_sample_pos(&timer_tmp);
    set_next_timer_settings(&timer_tmp);

    sys_unlock_cnt();

    // Calibrate current offset
    ENABLE_GATE()
    ; DCCAL_OFF();
    do_dc_cal();

    // Various time measurements
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
    PrescalerValue = (uint16_t)static_cast<float>(TIM12_CLOCK / TIM12_FREQ) - 1;

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM12, ENABLE);

    // Start threads
    timer_thd_stop = false;
    rpm_thd_stop = false;
    chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO,
                      timer_thread, NULL);
    chThdCreateStatic(rpm_thread_wa, sizeof(rpm_thread_wa), NORMALPRIO,
                      rpm_thread, NULL);

    // WWDG configuration
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
    WWDG_SetPrescaler(WWDG_Prescaler_1);
    WWDG_SetWindowValue(255);
    WWDG_Enable(100);

    // Reset tachometers again
    m_tachometer = 0;
    m_tachometer_abs = 0;

    m_init_done = true;
  }

  void deinit(void) {
    m_init_done = false;

    WWDG_DeInit();

    timer_thd_stop = true;
    rpm_thd_stop = true;

    while (timer_thd_stop || rpm_thd_stop) {
      chThdSleepMilliseconds(1);
    }

    TIM_DeInit(TIM1);
    TIM_DeInit(TIM2);
    TIM_DeInit(TIM8);
    TIM_DeInit(TIM12);
    ADC_DeInit();
    DMA_DeInit(DMA2_Stream4);
    nvicDisableVector(ADC_IRQn);
    dmaStreamRelease(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)));
  }

  bool init_done(void) {
    return m_init_done;
  }

  void set_configuration(mc_configuration *configuration) {
    // Stop everything first to be safe
    m_control_mode = CONTROL_MODE_NONE;
    stop_pwm_ll();

    sys_lock_cnt();
    m_conf = configuration;
    m_comm_mode_next = m_conf->comm_mode;
    init_hall_table((int8_t*)m_conf->hall_table);
    update_sensor_mode();
    sys_unlock_cnt();
  }

  /**
   * Initialize the hall sensor lookup table
   *
   * @param table
   * The commutations corresponding to the hall sensor states in the forward direction-
   */
  void init_hall_table(int8_t *table) {
    const int fwd_to_rev[7] = {-1, 1, 6, 5, 4, 3, 2};

    for (int i = 0; i < 8; i++) {
      m_hall_to_phase_table[8 + i] = table[i];
      int ind_now = m_hall_to_phase_table[8 + i];

      if (ind_now < 1) {
        m_hall_to_phase_table[i] = ind_now;
        continue;
      }

      m_hall_to_phase_table[i] = fwd_to_rev[ind_now];
    }
  }

  void do_dc_cal(void) {
    DCCAL_ON();

    // Wait max 5 seconds
    int cnt = 0;
    while (IS_DRV_FAULT()) {
      chThdSleepMilliseconds(1);
      cnt++;
      if (cnt > 5000) {
        break;
      }
    };

    chThdSleepMilliseconds(1000);
    m_curr0_sum = 0;
    m_curr1_sum = 0;

#ifdef HW_HAS_3_SHUNTS
    m_curr2_sum = 0;
#endif

    m_curr_start_samples = 0;
    while (m_curr_start_samples < 4000) {
    };
    m_curr0_offset = m_curr0_sum / m_curr_start_samples;
    m_curr1_offset = m_curr1_sum / m_curr_start_samples;

#ifdef HW_HAS_3_SHUNTS
    m_curr2_offset = m_curr2_sum / m_curr_start_samples;
#endif

    DCCAL_OFF();
    m_dccal_done = true;
  }

  /**
   * Use duty cycle control. Absolute values less than MIN_DUTY_CYCLE will
   * stop the motor.
   *
   * @param dutyCycle
   * The duty cycle to use.
   */
  void set_duty(float dutyCycle) {
    m_control_mode = CONTROL_MODE_DUTY;
    set_duty_cycle_hl(dutyCycle);
  }

  /**
   * Use duty cycle control. Absolute values less than MIN_DUTY_CYCLE will
   * stop the motor.
   *
   * WARNING: This function does not use ramping. A too large step with a large motor
   * can destroy hardware.
   *
   * @param dutyCycle
   * The duty cycle to use.
   */
  void set_duty_noramp(float dutyCycle) {
    m_control_mode = CONTROL_MODE_DUTY;

    if (m_state != MC_STATE_RUNNING) {
      set_duty_cycle_hl(dutyCycle);
    }
    else {
      m_dutycycle_set = dutyCycle;
      m_dutycycle_now = dutyCycle;
      set_duty_cycle_ll(dutyCycle);
    }
  }

  /**
   * Use PID rpm control. Note that this value has to be multiplied by half of
   * the number of motor poles.
   *
   * @param rpm
   * The electrical RPM goal value to use.
   */
  void set_pid_speed(float rpm) {
    m_control_mode = CONTROL_MODE_SPEED;
    m_speed_pid_set_rpm = rpm;
  }

  /**
   * Use PID position control. Note that this only works when encoder support
   * is enabled.
   *
   * @param pos
   * The desired position of the motor in degrees.
   */
  void set_pid_pos(float pos) {
    m_control_mode = CONTROL_MODE_POS;
    m_pos_pid_set_pos = pos;

    if (m_state != MC_STATE_RUNNING) {
      set_duty_cycle_hl(m_conf->l_min_duty);
    }
  }

  /**
   * Use current control and specify a goal current to use. The sign determines
   * the direction of the torque. Absolute values less than
   * conf->cc_min_current will release the motor.
   *
   * @param current
   * The current to use.
   */
  void set_current(ampere_t current) {
    if (fabsf(current) < m_conf->cc_min_current) {
      m_control_mode = CONTROL_MODE_NONE;
      stop_pwm_ll();
      return;
    }

    truncate_number_abs(current, m_conf->l_current_max);

    m_control_mode = CONTROL_MODE_CURRENT;
    m_current_set = current;

    if (m_state != MC_STATE_RUNNING) {
      set_duty_cycle_hl(SIGN(current) * m_conf->l_min_duty);
    }
  }

  /**
   * Brake the motor with a desired current. Absolute values less than
   * conf->cc_min_current will release the motor.
   *
   * @param current
   * The current to use. Positive and negative values give the same effect.
   */
  void set_brake_current(ampere_t current) {
    if (fabsf(current) < m_conf->cc_min_current) {
      m_control_mode = CONTROL_MODE_NONE;
      stop_pwm_ll();
      return;
    }

    truncate_number_abs(current, fabsf(m_conf->l_current_min));

    m_control_mode = CONTROL_MODE_CURRENT_BRAKE;
    m_current_set = current;

    if (m_state != MC_STATE_RUNNING && m_state != MC_STATE_FULL_BRAKE) {
      // In case the motor is already spinning, set the state to running
      // so that it can be ramped down before the full brake is applied.

      if (fabsf(m_rpm_now) > m_conf->l_max_erpm_fbrake) {
        m_state = MC_STATE_RUNNING;
      }
      else {
        full_brake_ll();
      }
    }
  }

  /**
   * Get the electrical position (or commutation step) of the motor.
   *
   * @return
   * The current commutation step. Range [1 6]
   */
  int get_comm_step(void) {
    return m_comm_step;
  }

  float get_duty_cycle_set(void) {
    return m_dutycycle_set;
  }

  float get_duty_cycle_now(void) {
    return m_dutycycle_now;
  }

  /**
   * Get the current switching frequency.
   *
   * @return
   * The switching frequency in Hz.
   */
  hertz_t get_switching_frequency_now(void) {
    return m_switching_frequency_now;
  }

  /**
   * Calculate the current RPM of the motor. This is a signed value and the sign
   * depends on the direction the motor is rotating in. Note that this value has
   * to be divided by half the number of motor poles.
   *
   * @return
   * The RPM value.
   */
  float get_rpm(void) {
    return m_direction ? m_rpm_now : -m_rpm_now;
  }

  mc_state get_state(void) {
    return m_state;
  }

  /**
   * Calculate the KV (RPM per volt) value for the motor. This function has to
   * be used while the motor is moving. Note that the return value has to be
   * divided by half the number of motor poles.
   *
   * @return
   * The KV value.
   */
  float get_kv(void) {
    return m_rpm_now / (GET_INPUT_VOLTAGE() * fabsf(m_dutycycle_now));
  }

  /**
   * Calculate the FIR-filtered KV (RPM per volt) value for the motor. This
   * function has to be used while the motor is moving. Note that the return
   * value has to be divided by half the number of motor poles.
   *
   * @return
   * The filtered KV value.
   */
  float get_kv_filtered(void) {
    float value = filter::run_fir_iteration((float*)m_kv_fir_samples,
                                            (float*)m_kv_fir_coeffs,
                                            KV_FIR_TAPS_BITS,
                                            m_kv_fir_index);

    return value;
  }

  /**
   * Get the motor current. The sign of this value will
   * represent whether the motor is drawing (positive) or generating
   * (negative) current.
   *
   * @return
   * The motor current.
   */
  ampere_t get_tot_current(void) {
    return m_last_current_sample;
  }

  /**
   * Get the FIR-filtered motor current. The sign of this value will
   * represent whether the motor is drawing (positive) or generating
   * (negative) current.
   *
   * @return
   * The filtered motor current.
   */
  ampere_t get_tot_current_filtered(void) {
    return m_last_current_sample_filtered;
  }

  /**
   * Get the motor current. The sign of this value represents the direction
   * in which the motor generates torque.
   *
   * @return
   * The motor current.
   */
  ampere_t get_tot_current_directional(void) {
    auto const retval = get_tot_current();
    return m_dutycycle_now > 0.0 ? retval : -retval;
  }

  /**
   * Get the filtered motor current. The sign of this value represents the
   * direction in which the motor generates torque.
   *
   * @return
   * The filtered motor current.
   */
  ampere_t get_tot_current_directional_filtered(void) {
    auto const retval = get_tot_current_filtered();
    return m_dutycycle_now > 0.0 ? retval : -retval;
  }

  /**
   * Get the input current to the motor controller.
   *
   * @return
   * The input current.
   */
  ampere_t get_tot_current_in(void) {
    return get_tot_current() * fabsf(m_dutycycle_now);
  }

  /**
   * Get the FIR-filtered input current to the motor controller.
   *
   * @return
   * The filtered input current.
   */
  ampere_t get_tot_current_in_filtered(void) {
    return get_tot_current_filtered() * fabsf(m_dutycycle_now);
  }

  /**
   * Read the number of steps the motor has rotated. This number is signed and
   * will return a negative number when the motor is rotating backwards.
   *
   * @param reset
   * If true, the tachometer counter will be reset after this call.
   *
   * @return
   * The tachometer value in motor steps. The number of motor revolutions will
   * be this number divided by (3 * MOTOR_POLE_NUMBER).
   */
  int get_tachometer_value(bool reset) {
    int val = m_tachometer;

    if (reset) {
      m_tachometer = 0;
    }

    return val;
  }

  /**
   * Read the absolute number of steps the motor has rotated.
   *
   * @param reset
   * If true, the tachometer counter will be reset after this call.
   *
   * @return
   * The tachometer value in motor steps. The number of motor revolutions will
   * be this number divided by (3 * MOTOR_POLE_NUMBER).
   */
  int get_tachometer_abs_value(bool reset) {
    int val = m_tachometer_abs;

    if (reset) {
      m_tachometer_abs = 0;
    }

    return val;
  }

  /**
   * Switch off all FETs.
   */
  void stop_pwm(void) {
    m_control_mode = CONTROL_MODE_NONE;
    stop_pwm_ll();
  }

  void stop_pwm_ll(void) {
    m_state = MC_STATE_OFF;
    m_ignore_iterations = CMD_STOP_TIME;
    stop_pwm_hw();
  }

  void stop_pwm_hw(void) {
#ifdef HW_HAS_DRV8313
    DISABLE_BR()
    ;
#endif

    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

    TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

    TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

    set_switching_frequency(m_conf->m_bldc_f_sw_max);
  }

  void full_brake_ll(void) {
    m_state = MC_STATE_FULL_BRAKE;
    m_ignore_iterations = CMD_STOP_TIME;
    full_brake_hw();
  }

  void full_brake_hw(void) {
#ifdef HW_HAS_DRV8313
    ENABLE_BR()
    ;
#endif

    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

    TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

    TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

    set_switching_frequency(m_conf->m_bldc_f_sw_max);
  }

  /**
   * High-level duty cycle setter. Will set the ramping goal of the duty cycle.
   * If motor is not running, it will be started in different ways depending on
   * whether it is moving or not.
   *
   * @param dutyCycle
   * The duty cycle in the range [-MAX_DUTY_CYCLE MAX_DUTY_CYCLE]
   * If the absolute value of the duty cycle is less than MIN_DUTY_CYCLE,
   * the motor phases will be shorted to brake the motor.
   */
  void set_duty_cycle_hl(float dutyCycle) {
    truncate_number_abs(dutyCycle, m_conf->l_max_duty);

    if (m_state == MC_STATE_DETECTING) {
      stop_pwm_ll();
      return;
    }

    m_dutycycle_set = dutyCycle;

    if (m_state != MC_STATE_RUNNING) {
      if (fabsf(dutyCycle) >= m_conf->l_min_duty) {
        // dutycycle_now is updated by the back-emf detection. If the motor already
        // is spinning, it will be non-zero.
        if (fabsf(m_dutycycle_now) < m_conf->l_min_duty) {
          m_dutycycle_now = SIGN(dutyCycle) * m_conf->l_min_duty;
        }

        set_duty_cycle_ll(m_dutycycle_now);
      }
      else {
        // In case the motor is already spinning, set the state to running
        // so that it can be ramped down before the full brake is applied.
        if (fabsf(m_rpm_now) > m_conf->l_max_erpm_fbrake) {
          m_state = MC_STATE_RUNNING;
        }
        else {
          full_brake_ll();
        }
      }
    }
  }

  /**
   * Low-level duty cycle setter. Will update the state of the application
   * and the motor direction accordingly.
   *
   * This function should be used with care. Ramping together with current
   * limiting should be used.
   *
   * @param dutyCycle
   * The duty cycle in the range [-MAX_DUTY_CYCLE MAX_DUTY_CYCLE]
   * If the absolute value of the duty cycle is less than MIN_DUTY_CYCLE,
   * the motor will be switched off.
   */
  void set_duty_cycle_ll(float dutyCycle) {
    if (dutyCycle >= m_conf->l_min_duty) {
      m_direction = 1;
    }
    else if (dutyCycle <= -m_conf->l_min_duty) {
      dutyCycle = -dutyCycle;
      m_direction = 0;
    }

    if (dutyCycle < m_conf->l_min_duty) {
      float max_erpm_fbrake;
#if BLDC_SPEED_CONTROL_CURRENT
      if (m_control_mode == CONTROL_MODE_CURRENT
          || m_control_mode == CONTROL_MODE_CURRENT_BRAKE
          || m_control_mode == CONTROL_MODE_SPEED) {
#else
        if (m_control_mode == CONTROL_MODE_CURRENT ||
            m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
#endif
        max_erpm_fbrake = m_conf->l_max_erpm_fbrake_cc;
      }
      else {
        max_erpm_fbrake = m_conf->l_max_erpm_fbrake;
      }

      switch (m_state) {
      case MC_STATE_RUNNING:
        // TODO!!!
        if (fabsf(m_rpm_now) > max_erpm_fbrake) {
          dutyCycle = m_conf->l_min_duty;
        }
        else {
          full_brake_ll();
          return;
        }
        break;

      case MC_STATE_DETECTING:
        stop_pwm_ll();
        return;
        break;

      default:
        return;
      }
    }
    else if (dutyCycle > m_conf->l_max_duty) {
      dutyCycle = m_conf->l_max_duty;
    }

    set_duty_cycle_hw(dutyCycle);

    if (m_sensorless_now) {
      if (m_state != MC_STATE_RUNNING) {
        if (m_state == MC_STATE_OFF) {
          m_state = MC_STATE_RUNNING;

          if (fabsf(m_rpm_now) < m_conf->sl_min_erpm) {
            commutate(1);
          }
        }
        else if (m_state == MC_STATE_FULL_BRAKE) {
          if (fabsf(m_rpm_now) < m_conf->sl_min_erpm
              && get_tot_current_filtered()
                  < m_conf->sl_max_fullbreak_current_dir_change) {
            m_state = MC_STATE_RUNNING;
            commutate(1);
          }
        }
      }
    }
    else {
      if (m_state != MC_STATE_RUNNING) {
        m_state = MC_STATE_RUNNING;
        m_comm_step = read_hall_phase();
        set_next_comm_step(m_comm_step);
        commutate(1);
      }
    }
  }

  /**
   * Lowest level (hardware) duty cycle setter. Will set the hardware timer to
   * the specified duty cycle and update the ADC sampling positions.
   *
   * @param dutyCycle
   * The duty cycle in the range [MIN_DUTY_CYCLE  MAX_DUTY_CYCLE]
   * (Only positive)
   */
  void set_duty_cycle_hw(float dutyCycle) {
    MCTimer timer_tmp;

    sys_lock_cnt();
    timer_tmp = m_timer_struct;
    sys_unlock_cnt();

    truncate_number(dutyCycle, m_conf->l_min_duty, m_conf->l_max_duty);

    if (is_detecting() || m_conf->pwm_mode == PWM_MODE_BIPOLAR) {
      m_switching_frequency_now = m_conf->m_bldc_f_sw_max;
    }
    else {
      m_switching_frequency_now = m_conf->m_bldc_f_sw_min
          * (1.0 - fabsf(dutyCycle))
          + m_conf->m_bldc_f_sw_max * fabsf(dutyCycle);
    }

    timer_tmp.top = SYSTEM_CORE_CLOCK / (int)static_cast<float>(m_switching_frequency_now);

    if (m_conf->pwm_mode == PWM_MODE_BIPOLAR && !is_detecting()) {
      timer_tmp.duty = (uint16_t)(
          ((float)timer_tmp.top / 2.0) * dutyCycle
              + ((float)timer_tmp.top / 2.0));
    }
    else {
      timer_tmp.duty = (uint16_t)((float)timer_tmp.top * dutyCycle);
    }

    update_adc_sample_pos(&timer_tmp);
    set_next_timer_settings(&timer_tmp);
  }

  void run_pid_control_speed(void) {
    static float i_term = 0;
    static float prev_error = 0;
    float p_term;
    float d_term;

    // PID is off. Return.
    if (m_control_mode != CONTROL_MODE_SPEED) {
#if BLDC_SPEED_CONTROL_CURRENT
      i_term = 0.0;
#else
      i_term = m_dutycycle_now;
#endif
      prev_error = 0.0;
      return;
    }

    const float rpm = get_rpm();
    float error = m_speed_pid_set_rpm - rpm;

    // Too low RPM set. Stop and return.
    if (fabsf(m_speed_pid_set_rpm) < m_conf->s_pid_min_erpm) {
      i_term = m_dutycycle_now;
      prev_error = error;
      set_duty(0.0);
      return;
    }

#if BLDC_SPEED_CONTROL_CURRENT
    // Compute parameters
    p_term = error * m_conf->s_pid_kp * (1.0 / 20.0);
    i_term += error * (m_conf->s_pid_ki * PID_TIME_K) * (1.0 / 20.0);
    d_term = (error - prev_error) * (m_conf->s_pid_kd / PID_TIME_K)
        * (1.0 / 20.0);

    // Filter D
    static float d_filter = 0.0;
    UTILS_LP_FAST(d_filter, d_term, m_conf->p_pid_kd_filter);
    d_term = d_filter;

    // I-term wind-up protection
    truncate_number_abs(i_term, 1.0);

    // Store previous error
    prev_error = error;

    // Calculate output
    float output = p_term + i_term + d_term;
    truncate_number_abs(output, 1.0);

    // Optionally disable braking
    if (!m_conf->s_pid_allow_braking) {
      if (rpm > 0.0 && output < 0.0) {
        output = 0.0;
      }

      if (rpm < 0.0 && output > 0.0) {
        output = 0.0;
      }
    }

    m_current_set = output * m_conf->lo_current_max;

    if (m_state != MC_STATE_RUNNING) {
      set_duty_cycle_hl(SIGN(output) * m_conf->l_min_duty);
    }
#else
    // Compensation for supply voltage variations
    float scale = 1.0 / GET_INPUT_VOLTAGE();

    // Compute parameters
    p_term = error * m_conf->s_pid_kp * scale;
    i_term += error * (m_conf->s_pid_ki * PID_TIME_K) * scale;
    d_term = (error - prev_error) * (m_conf->s_pid_kd / PID_TIME_K) * scale;

    // Filter D
    static float d_filter = 0.0;
    UTILS_LP_FAST(d_filter, d_term, m_conf->s_pid_kd_filter);
    d_term = d_filter;

    // I-term wind-up protection
    truncate_number_abs(i_term, 1.0);

    // Store previous error
    prev_error = error;

    // Calculate output
    float output = p_term + i_term + d_term;

    // Make sure that at least minimum output is used
    if (fabsf(output) < m_conf->l_min_duty) {
      output = SIGN(output) * m_conf->l_min_duty;
    }

    // Do not output in reverse direction to oppose too high rpm
    if (m_speed_pid_set_rpm > 0.0 && output < 0.0) {
      output = m_conf->l_min_duty;
      i_term = 0.0;
    }
    else if (m_speed_pid_set_rpm < 0.0 && output > 0.0) {
      output = -m_conf->l_min_duty;
      i_term = 0.0;
    }

    set_duty_cycle_hl(output);
#endif
  }

  void run_pid_control_pos(second_t const _dt) {

    auto const dt = static_cast<float>(_dt);
    static float i_term = 0;
    static float prev_error = 0;
    float p_term;
    float d_term;

    // PID is off. Return.
    if (m_control_mode != CONTROL_MODE_POS) {
      i_term = 0;
      prev_error = 0;
      return;
    }

    // Compute error
    float error = angle_difference(encoder::read_deg(), m_pos_pid_set_pos);

    // Compute parameters
    p_term = error * m_conf->p_pid_kp;
    i_term += error * (m_conf->p_pid_ki * dt);
    d_term = (error - prev_error) * (m_conf->p_pid_kd / dt);

    // Filter D
    static float d_filter = 0.0;
    UTILS_LP_FAST(d_filter, d_term, m_conf->p_pid_kd_filter);
    d_term = d_filter;

    // I-term wind-up protection
    truncate_number_abs(i_term, 1.0);

    // Store previous error
    prev_error = error;

    // Calculate output
    float output = p_term + i_term + d_term;
    truncate_number_abs(output, 1.0);

    m_current_set = output * m_conf->lo_current_max;
  }

  THD_FUNCTION(rpm_thread, arg) {
    (void)arg;

    chRegSetThreadName("rpm timer");

    for (;;) {
      if (rpm_thd_stop) {
        rpm_thd_stop = false;
        return;
      }

      if (rpm_dep.comms != 0) {
        // GG: commutations occurred since previous rpm measure
        sys_lock_cnt();
        const float comms = (float)rpm_dep.comms;
        const float time_at_comm = (float)rpm_dep.time_at_comm;
        rpm_dep.comms = 0;
        rpm_dep.time_at_comm = 0;
        sys_unlock_cnt();

        m_rpm_now = (comms * static_cast<float>(RPM_TIMER_FREQ) * 60.0) / (time_at_comm * 6.0);      
      }
      else {
        // GG: still on the same commutation as in previous rpm evaluation
        // In case we have slowed down
        float rpm_tmp = (static_cast<float>(RPM_TIMER_FREQ) * 60.0) / ((float) TIM2->CNT * 6.0);
        if (fabsf(rpm_tmp) < fabsf(m_rpm_now)) {
          m_rpm_now = rpm_tmp;
        }
      }

      // Some low-pass filtering
      static float rpm_filtered = 0.0;
      UTILS_LP_FAST(rpm_filtered, m_rpm_now, 0.1);
      m_rpm_now = rpm_filtered;
      const float rpm_abs = fabsf(m_rpm_now);

      // Update the cycle integrator limit
      rpm_dep.cycle_int_limit = m_conf->sl_cycle_int_limit;
      rpm_dep.cycle_int_limit_running = rpm_dep.cycle_int_limit
          + (float)CONV_ADC_V(ADC_Value[ADC_IND_VIN_SENS])
              * m_conf->sl_bemf_coupling_k
              / (rpm_abs > m_conf->sl_min_erpm ? rpm_abs : m_conf->sl_min_erpm);

      rpm_dep.cycle_int_limit_running = map(
          rpm_abs, 0, m_conf->sl_cycle_int_rpm_br,
          rpm_dep.cycle_int_limit_running,
          rpm_dep.cycle_int_limit_running * m_conf->sl_phase_advance_at_br);

      rpm_dep.cycle_int_limit_max = rpm_dep.cycle_int_limit
          + (float)CONV_ADC_V(ADC_Value[ADC_IND_VIN_SENS])
              * m_conf->sl_bemf_coupling_k
              / m_conf->sl_min_erpm_cycle_int_limit;

      if (rpm_dep.cycle_int_limit_running < 1.0) {
        rpm_dep.cycle_int_limit_running = 1.0;
      }

      if (rpm_dep.cycle_int_limit_running > rpm_dep.cycle_int_limit_max) {
        rpm_dep.cycle_int_limit_running = rpm_dep.cycle_int_limit_max;
      }

      rpm_dep.comm_time_sum = static_cast<float>(m_conf->m_bldc_f_sw_max)
          / ((rpm_abs / 60.0) * 6.0);
      rpm_dep.comm_time_sum_min_rpm = static_cast<float>(m_conf->m_bldc_f_sw_max)
          / ((m_conf->sl_min_erpm / 60.0) * 6.0);

      run_pid_control_speed();

      chThdSleepMilliseconds(1);
    }
  }

  THD_FUNCTION(timer_thread, arg) {
    (void)arg;

    chRegSetThreadName("mcpwm timer");

    float amp;
    float min_s;
    float max_s;

    for (;;) {
      if (timer_thd_stop) {
        timer_thd_stop = false;
        return;
      }

      if (m_state == MC_STATE_OFF) {
        // Track the motor back-emf and follow it with dutycycle_now. Also track
        // the direction of the motor.
        amp = filter::run_fir_iteration((float*)m_amp_fir_samples,
                                        (float*)m_amp_fir_coeffs,
                                        AMP_FIR_TAPS_BITS,
                                        m_amp_fir_index);

        // Direction tracking
        if (m_sensorless_now) {
          min_s = 9999999999999.0;
          max_s = 0.0;

          for (int i = 0; i < 6; i++) {
            if (m_last_pwm_cycles_sums[i] < min_s) {
              min_s = m_last_pwm_cycles_sums[i];
            }

            if (m_last_pwm_cycles_sums[i] > max_s) {
              max_s = m_last_pwm_cycles_sums[i];
            }
          }

          // If the relative difference between the longest and shortest commutation is
          // too large, we probably got the direction wrong. In that case, try the other
          // direction.
          //
          // The tachometer_for_direction value is used to make sure that the samples
          // have enough time after a direction change to get stable before trying to
          // change direction again.

          if ((max_s - min_s) / ((max_s + min_s) / 2.0) > 1.2) {
            if (m_tachometer_for_direction > 12) {
              if (m_direction == 1) {
                m_direction = 0;
              }
              else {
                m_direction = 1;
              }
              m_tachometer_for_direction = 0;
            }
          }
          else {
            m_tachometer_for_direction = 0;
          }
        }
        else { // sensorless_now
          // If the direction tachometer is counting backwards, the motor is
          // not moving in the direction we think it is.
          if (m_tachometer_for_direction < -3) {
            if (m_direction == 1) {
              m_direction = 0;
            }
            else {
              m_direction = 1;
            }
            m_tachometer_for_direction = 0;
          }
          else if (m_tachometer_for_direction > 0) {
            m_tachometer_for_direction = 0;
          }
        }

        if (m_direction == 1) {
          m_dutycycle_now =
              amp / (float)CONV_ADC_V(ADC_Value[ADC_IND_VIN_SENS]);
        }
        else {
          m_dutycycle_now = -amp
              / (float)CONV_ADC_V(ADC_Value[ADC_IND_VIN_SENS]);
        }
        truncate_number_abs((float&)m_dutycycle_now, m_conf->l_max_duty);
      }
      else { //state != MC_STATE_OFF
        m_tachometer_for_direction = 0;
      }

      // Fill KV filter vector at 100Hz
      static int cnt_tmp = 0;
      cnt_tmp++;
      if (cnt_tmp >= 10) {
        cnt_tmp = 0;
        if (m_state == MC_STATE_RUNNING ||
           (m_state == MC_STATE_OFF && m_dutycycle_now >= m_conf->l_min_duty)) {
          filter::add_sample((float*)m_kv_fir_samples,
                             get_kv(),
                             KV_FIR_TAPS_BITS,
                             (uint32_t&)m_kv_fir_index);

        }
      }

      chThdSleepMilliseconds(1);
    }
  }

  void process_detecting() {
    static int detect_now = 0;

    if (detect_now == 4) {
      const float a = fabsf(ADC_curr_norm_value[0]);
      const float b = fabsf(ADC_curr_norm_value[1]);

      if (a > b) {
        detect_currents[m_detect_step] = a;
      }
      else {
        detect_currents[m_detect_step] = b;
      }

      if (m_detect_step > 0) {
        detect_currents_diff[m_detect_step] = detect_currents[m_detect_step - 1]
                                            - detect_currents[m_detect_step];
      }
      else {
        detect_currents_diff[m_detect_step] = detect_currents[5]
                                            - detect_currents[m_detect_step];
      }

      const int vzero = ADC_V_ZERO;
      //          const int vzero = (ADC_V_L1 + ADC_V_L2 + ADC_V_L3) / 3;

      switch (m_comm_step) {
      case 1:
      case 4:
        detect_voltages[m_detect_step] = ADC_V_L1 - vzero;
        break;

        case 2:
        case 5:
        detect_voltages[m_detect_step] = ADC_V_L2 - vzero;
        break;

        case 3:
        case 6:
        detect_voltages[m_detect_step] = ADC_V_L3 - vzero;
        break;

        default:
        break;
      }

      m_detect_currents_avg[m_detect_step] += detect_currents[m_detect_step];
      m_detect_avg_samples[m_detect_step]++;

      stop_pwm_hw();
    }

    if (detect_now) {
      detect_now--;
    }

    if (is_detecting() && detect_now == 0) {
      detect_now = 5;

      set_duty_cycle_hw(0.2);

      m_detect_step++;
      if (m_detect_step > 5) {
        m_detect_step = 0;
      }

      m_comm_step = m_detect_step + 1;

      set_next_comm_step(m_comm_step);
      TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    }
  }

  void adc_inj_int_handler(void) {
    TIM12->CNT = 0;

    int curr0 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    int curr1 = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);

    int curr0_2 = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_2);
    int curr1_2 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);

#ifdef HW_HAS_3_SHUNTS
    int curr2 = ADC_GetInjectedConversionValue(ADC3, ADC_InjectedChannel_1);
#endif

    float curr0_currsamp = curr0;
    float curr1_currsamp = curr1;
#ifdef HW_HAS_3_SHUNTS
    float curr2_currsamp = curr2;
#endif

    if (m_use_curr_samp_volt & (1 << 0)) {
      curr0 = ADC_Value[ADC_IND_CURR1];
    }

    if (m_use_curr_samp_volt & (1 << 1)) {
      curr1 = ADC_Value[ADC_IND_CURR2];
    }

#ifdef HW_HAS_3_SHUNTS
    if (m_use_curr_samp_volt & (1 << 2)) {
      curr2 = ADC_Value[ADC_IND_CURR3];
    }
#endif

    // DCCal every other cycle
    //	static bool sample_ofs = true;
    //	if (sample_ofs) {
    //		sample_ofs = false;
    //		curr0_offset = curr0;
    //		curr1_offset = curr1;
    //		DCCAL_OFF();
    //		return;
    //	} else {
    //		sample_ofs = true;
    //		DCCAL_ON();
    //	}

    m_curr0_sum += curr0;
    m_curr1_sum += curr1;
#ifdef HW_HAS_3_SHUNTS
    m_curr2_sum += curr2;
#endif

    m_curr_start_samples++;

    curr0_currsamp -= m_curr0_offset;
    curr1_currsamp -= m_curr1_offset;
    curr0 -= m_curr0_offset;
    curr1 -= m_curr1_offset;
    curr0_2 -= m_curr0_offset;
    curr1_2 -= m_curr1_offset;

#ifdef HW_HAS_3_SHUNTS
    curr2_currsamp -= m_curr2_offset;
    curr2 -= m_curr2_offset;
#endif

#if CURR1_DOUBLE_SAMPLE || CURR2_DOUBLE_SAMPLE
    if (m_conf->pwm_mode != PWM_MODE_BIPOLAR) {
      if (m_direction) {
        if (CURR1_DOUBLE_SAMPLE && m_comm_step == 3) {
          curr0 = (curr0 + curr0_2) / 2.0;
        }
        else if (CURR2_DOUBLE_SAMPLE && m_comm_step == 4) {
          curr1 = (curr1 + curr1_2) / 2.0;
        }
      }
      else {
        if (CURR1_DOUBLE_SAMPLE && m_comm_step == 2) {
          curr0 = (curr0 + curr0_2) / 2.0;
        }
        else if (CURR2_DOUBLE_SAMPLE && m_comm_step == 1) {
          curr1 = (curr1 + curr1_2) / 2.0;
        }
      }
    }
#endif

    ADC_curr_norm_value[0] = curr0;
    ADC_curr_norm_value[1] = curr1;

#ifdef HW_HAS_3_SHUNTS
    ADC_curr_norm_value[2] = curr2;
#else
    ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
#endif

#ifdef HW_IS_IHM0xM1
    ADC_curr_norm_value[0] *= -1;
    ADC_curr_norm_value[1] *= -1;
    ADC_curr_norm_value[2] *= -1;
#endif

    float curr_tot_sample = 0;

    /*
     * Commutation Steps FORWARDS
     * STEP		BR1		BR2		BR3
     * 1		0		+		-
     * 2		+		0		-
     * 3		+		-		0
     * 4		0		-		+
     * 5		-		0		+
     * 6		-		+		0
     *
     * Commutation Steps REVERSE (switch phase 2 and 3)
     * STEP		BR1		BR2		BR3
     * 1		0		-		+
     * 2		+		-		0
     * 3		+		0		-
     * 4		0		+		-
     * 5		-		+		0
     * 6		-		0		+
     */

    if (m_state == MC_STATE_FULL_BRAKE) {
      float c0 = (float)ADC_curr_norm_value[0];
      float c1 = (float)ADC_curr_norm_value[1];
      float c2 = (float)ADC_curr_norm_value[2];
      curr_tot_sample = sqrtf((c0 * c0 + c1 * c1 + c2 * c2) / 1.5);
    }
    else {
#ifdef HW_HAS_3_SHUNTS
      if (m_direction) {
        switch (m_comm_step) {
        case 1:
          curr_tot_sample = -(float)ADC_curr_norm_value[2];
          break;
        case 2:
          curr_tot_sample = -(float)ADC_curr_norm_value[2];
          break;
        case 3:
          curr_tot_sample = -(float)ADC_curr_norm_value[1];
          break;
        case 4:
          curr_tot_sample = -(float)ADC_curr_norm_value[1];
          break;
        case 5:
          curr_tot_sample = -(float)ADC_curr_norm_value[0];
          break;
        case 6:
          curr_tot_sample = -(float)ADC_curr_norm_value[0];
          break;
        default:
          break;
        }
      }
      else {
        switch (m_comm_step) {
        case 1:
          curr_tot_sample = -(float)ADC_curr_norm_value[1];
          break;
        case 2:
          curr_tot_sample = -(float)ADC_curr_norm_value[1];
          break;
        case 3:
          curr_tot_sample = -(float)ADC_curr_norm_value[2];
          break;
        case 4:
          curr_tot_sample = -(float)ADC_curr_norm_value[2];
          break;
        case 5:
          curr_tot_sample = -(float)ADC_curr_norm_value[0];
          break;
        case 6:
          curr_tot_sample = -(float)ADC_curr_norm_value[0];
          break;
        default:
          break;
        }
      }
#else
      if (m_direction) {
        switch (m_comm_step) {
          case 1: curr_tot_sample = -(float)ADC_curr_norm_value[1]; break;
          case 2: curr_tot_sample = -(float)ADC_curr_norm_value[1]; break;
          case 3: curr_tot_sample =  (float)ADC_curr_norm_value[0]; break;
          case 4: curr_tot_sample =  (float)ADC_curr_norm_value[1]; break;
          case 5: curr_tot_sample = -(float)ADC_curr_norm_value[0]; break;
          case 6: curr_tot_sample = -(float)ADC_curr_norm_value[0]; break;
          default: break;
        }
      }
      else {
        switch (m_comm_step) {
          case 1: curr_tot_sample =  (float)ADC_curr_norm_value[1]; break;
          case 2: curr_tot_sample =  (float)ADC_curr_norm_value[0]; break;
          case 3: curr_tot_sample = -(float)ADC_curr_norm_value[1]; break;
          case 4: curr_tot_sample = -(float)ADC_curr_norm_value[1]; break;
          case 5: curr_tot_sample = -(float)ADC_curr_norm_value[0]; break;
          case 6: curr_tot_sample = -(float)ADC_curr_norm_value[0]; break;
          default: break;
        }
      }
#endif

      const float tot_sample_tmp = curr_tot_sample;
      static int comm_step_prev = 1;
      static float prev_tot_sample = 0.0;
      if (m_comm_step != comm_step_prev) {
        curr_tot_sample = prev_tot_sample;
      }
      comm_step_prev = m_comm_step;
      prev_tot_sample = tot_sample_tmp;
    }

    process_detecting();

    m_last_current_sample = curr_tot_sample * FAC_CURRENT;

    // Filter out outliers
    if (fabsf(m_last_current_sample) > (m_conf->l_abs_current_max * 1.2)) {
      m_last_current_sample = SIGN(m_last_current_sample)* m_conf->l_abs_current_max * 1.2;
    }

    filter::add_sample((float*)m_current_fir_samples,
                       m_last_current_sample,
                       CURR_FIR_TAPS_BITS,
                       (uint32_t&)m_current_fir_index);

    m_last_current_sample_filtered = filter::run_fir_iteration((float*)m_current_fir_samples,
                                                               (float*)m_current_fir_coeffs,
                                                               CURR_FIR_TAPS_BITS,
                                                               m_current_fir_index);

    m_last_inj_adc_isr_duration = TIM12->CNT / TIM12_FREQ;
  }

  /*
   * New ADC samples ready. Do commutation!
   */
  void adc_int_handler(void *p, uint32_t flags) {
    (void)p;
    (void)flags;

    TIM12->CNT = 0;

    // Set the next timer settings if an update is far enough away
    update_timer_attempt();

    // Reset the watchdog
    WWDG_SetCounter(100);

    const float input_voltage = GET_INPUT_VOLTAGE();
    int ph1, ph2, ph3;
    int ph1_raw, ph2_raw, ph3_raw;

    static int direction_before = 1;
    if (!(m_state == MC_STATE_RUNNING && m_direction == direction_before)) {
      m_has_commutated = 0;
    }
    direction_before = m_direction;

    /*
     * Calculate the virtual ground, depending on the state.
     */
    if (m_has_commutated && fabsf(m_dutycycle_now) > 0.2) {
      mcpwm_vzero = ADC_V_ZERO;
    }
    else {
      mcpwm_vzero = (ADC_V_L1+ ADC_V_L2 + ADC_V_L3) / 3;
    }

    if (m_direction) {
      ph1 = ADC_V_L1- mcpwm_vzero;
      ph2 = ADC_V_L2 - mcpwm_vzero;
      ph3 = ADC_V_L3 - mcpwm_vzero;
      ph1_raw = ADC_V_L1;
      ph2_raw = ADC_V_L2;
      ph3_raw = ADC_V_L3;
    }
    else {
      ph1 = ADC_V_L1 - mcpwm_vzero;
      ph2 = ADC_V_L3 - mcpwm_vzero;
      ph3 = ADC_V_L2 - mcpwm_vzero;
      ph1_raw = ADC_V_L1;
      ph2_raw = ADC_V_L3;
      ph3_raw = ADC_V_L2;
    }

    update_timer_attempt();

    {
      float amp = 0.0;

      if (m_has_commutated) {
        amp = fabsf(
            m_dutycycle_now) * (float)CONV_ADC_V(ADC_Value[ADC_IND_VIN_SENS]);
      }
      else {
        amp = sqrtf((float)(ph1 * ph1 + ph2 * ph2 + ph3 * ph3)) * sqrtf(2.0);
      }

      // Fill the amplitude FIR filter
      filter::add_sample((float*)m_amp_fir_samples,
                         amp,
                         AMP_FIR_TAPS_BITS,
                         (uint32_t&)
                         m_amp_fir_index);
    }

    if (m_sensorless_now) {
      static float cycle_integrator = 0;

      if (m_pwm_cycles_sum >= rpm_dep.comm_time_sum_min_rpm) {
        if (m_state == MC_STATE_RUNNING) {
          if (m_conf->comm_mode == COMM_MODE_INTEGRATE) {
            // This means that the motor is stuck. If this commutation does not
            // produce any torque because of misalignment at start, two
            // commutations ahead should produce full torque.
            commutate(2);
          }
          else if (m_conf->comm_mode == COMM_MODE_DELAY) {
            commutate(1);
          }

          cycle_integrator = 0.0;
        }
      }

      if ((m_state == MC_STATE_RUNNING && m_pwm_cycles >= 2)
          || m_state == MC_STATE_OFF) {
        int v_diff = 0;
        int ph_now_raw = 0;

        switch (m_comm_step) {
        case 1:
          v_diff = ph1;
          ph_now_raw = ph1_raw;
          break;
        case 2:
          v_diff = -ph2;
          ph_now_raw = ph2_raw;
          break;
        case 3:
          v_diff = ph3;
          ph_now_raw = ph3_raw;
          break;
        case 4:
          v_diff = -ph1;
          ph_now_raw = ph1_raw;
          break;
        case 5:
          v_diff = ph2;
          ph_now_raw = ph2_raw;
          break;
        case 6:
          v_diff = -ph3;
          ph_now_raw = ph3_raw;
          break;
        default:
          break;
        }

        // Collect hall sensor samples in the first half of the commutation cycle. This is
        // because positive timing is much better than negative timing in case they are
        // mis-aligned.
        if (v_diff < 50) {
          m_hall_detect_table[read_hall()][m_comm_step]++;
        }

        // Don't commutate while the motor is standing still and the signal only consists
        // of weak noise.
        if (abs(v_diff) < 10) {
          v_diff = 0;
        }

        if (v_diff > 0) {
          // TODO!
          //					const int min = 100;
          int min = (int)((1.0 - fabsf(m_dutycycle_now))
              * (float)CONV_ADC_V(ADC_Value[ADC_IND_VIN_SENS] * 0.3));
          if (min > CONV_ADC_V(ADC_Value[ADC_IND_VIN_SENS] / 4)) {
            min = CONV_ADC_V(ADC_Value[ADC_IND_VIN_SENS] / 4);
          }

          if (m_pwm_cycles_sum > (m_last_pwm_cycles_sum / 2.0)
              || !m_has_commutated
              || (ph_now_raw > min
                  && ph_now_raw
                      < (CONV_ADC_V(ADC_Value[ADC_IND_VIN_SENS]) - min))) {
            cycle_integrator += (float)v_diff / static_cast<float>(m_switching_frequency_now);
          }
        }

        static scalar_t cycle_sum = 0.0;
        if (m_conf->comm_mode == COMM_MODE_INTEGRATE) {
          float limit;
          if (m_has_commutated) {
            limit = rpm_dep.cycle_int_limit_running * (0.0005 * VDIV_CORR);
          }
          else {
            limit = rpm_dep.cycle_int_limit * (0.0005 * VDIV_CORR);
          }

          if (cycle_integrator
              >= (rpm_dep.cycle_int_limit_max * (0.0005 * VDIV_CORR))
              || cycle_integrator >= limit) {
            commutate(1);
            cycle_integrator = 0.0;
            cycle_sum = 0.0;
          }
        }
        else if (m_conf->comm_mode == COMM_MODE_DELAY) {
          if (v_diff > 0) {
            cycle_sum += m_conf->m_bldc_f_sw_max / m_switching_frequency_now;

            if (cycle_sum
                >= map(
                    fabsf(m_rpm_now),
                    0,
                    m_conf->sl_cycle_int_rpm_br,
                    rpm_dep.comm_time_sum / 2.0,
                    (rpm_dep.comm_time_sum / 2.0)
                        * m_conf->sl_phase_advance_at_br)) {
              commutate(1);
              m_cycle_integrator_sum += cycle_integrator
                  * (1.0 / (0.0005 * VDIV_CORR));
              m_cycle_integrator_iterations += 1.0;
              cycle_integrator = 0.0;
              cycle_sum = 0.0;
            }
          }
          else {
            cycle_integrator = 0.0;
            cycle_sum = 0.0;
          }
        }
      }
      else {
        cycle_integrator = 0.0;
      }

      m_pwm_cycles_sum += m_conf->m_bldc_f_sw_max / m_switching_frequency_now;
      m_pwm_cycles++;

    }
    else { // !sensorless_now
      const int hall_phase = read_hall_phase();
      if (m_comm_step != hall_phase) {
        m_comm_step = hall_phase;

        update_rpm_tacho();

        if (m_state == MC_STATE_RUNNING) {
          set_next_comm_step(m_comm_step);
          commutate(0);
        }
      }
      else if (m_state == MC_STATE_RUNNING && !m_has_commutated) {
        set_next_comm_step(m_comm_step);
        commutate(0);
      }
    }

    auto const current_nofilter = get_tot_current();
    auto const current_in_nofilter = current_nofilter * fabsf(m_dutycycle_now);

    if (m_state == MC_STATE_RUNNING && m_has_commutated) {
      // Compensation for supply voltage variations
      const float voltage_scale = 20.0 / input_voltage;
      float ramp_step = m_conf->m_duty_ramp_step
          / static_cast<float>(m_switching_frequency_now / 1000.0);
      float ramp_step_no_lim = ramp_step;
      const float rpm = get_rpm();

      if (m_slow_ramping_cycles) {
        m_slow_ramping_cycles--;
        ramp_step *= 0.1;
      }

      float dutycycle_now_tmp = m_dutycycle_now;

#if BLDC_SPEED_CONTROL_CURRENT
      if (m_control_mode == CONTROL_MODE_CURRENT
          || m_control_mode == CONTROL_MODE_POS
          || m_control_mode == CONTROL_MODE_SPEED) {
#else
        if (m_control_mode == CONTROL_MODE_CURRENT ||
            m_control_mode == CONTROL_MODE_POS) {
#endif
        // Compute error
        const float error = m_current_set
            - (m_direction ? current_nofilter : -current_nofilter);
        float step = error * m_conf->cc_gain * voltage_scale;
        const float start_boost = m_conf->cc_startup_boost_duty * voltage_scale;

        // Do not ramp too much
        truncate_number_abs(step, m_conf->cc_ramp_step_max);

        // Switching frequency correction
        step /= static_cast<float>(m_switching_frequency_now / 1000.0);

        if (m_slow_ramping_cycles) {
          m_slow_ramping_cycles--;
          step *= 0.1;
        }

        // Optionally apply startup boost.
        if (fabsf(dutycycle_now_tmp) < start_boost) {
          step_towards(dutycycle_now_tmp,
                       m_current_set > 0.0 ? start_boost : -start_boost,
                       ramp_step);
        }
        else {
          dutycycle_now_tmp += step;
        }

        // Upper truncation
        truncate_number_abs((float&)dutycycle_now_tmp, m_conf->l_max_duty);

        // Lower truncation
        if (fabsf(dutycycle_now_tmp) < m_conf->l_min_duty) {
          if (dutycycle_now_tmp < 0.0 && m_current_set > 0.0) {
            dutycycle_now_tmp = m_conf->l_min_duty;
          }
          else if (dutycycle_now_tmp > 0.0 && m_current_set < 0.0) {
            dutycycle_now_tmp = -m_conf->l_min_duty;
          }
        }

        // The set dutycycle should be in the correct direction in case the output is lower
        // than the minimum duty cycle and the mechanism below gets activated.
        m_dutycycle_set =
            dutycycle_now_tmp >= 0.0 ? m_conf->l_min_duty : -m_conf->l_min_duty;

      }
      else if (m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
        // Compute error
        const float error = -fabsf(m_current_set) - current_nofilter;
        float step = error * m_conf->cc_gain * voltage_scale;

        // Do not ramp too much
        truncate_number_abs(step, m_conf->cc_ramp_step_max);

        // Switching frequency correction
        step /= static_cast<float>(m_switching_frequency_now) / 1000.0;

        if (m_slow_ramping_cycles) {
          m_slow_ramping_cycles--;
          step *= 0.1;
        }

        dutycycle_now_tmp += SIGN(dutycycle_now_tmp) * step;

        // Upper truncation
        truncate_number_abs((float&)dutycycle_now_tmp, m_conf->l_max_duty);

        // Lower truncation
        if (fabsf(dutycycle_now_tmp) < m_conf->l_min_duty) {
          if (fabsf(m_rpm_now) < m_conf->l_max_erpm_fbrake_cc) {
            dutycycle_now_tmp = 0.0;
            m_dutycycle_set = dutycycle_now_tmp;
          }
          else {
            dutycycle_now_tmp = SIGN(dutycycle_now_tmp) * m_conf->l_min_duty;
            m_dutycycle_set = dutycycle_now_tmp;
          }
        }
      }
      else {
        step_towards((float&)dutycycle_now_tmp, m_dutycycle_set, ramp_step);
      }

      static int limit_delay = 0;

      // Apply limits in priority order
      if (current_nofilter > m_conf->lo_current_max) {
        step_towards(
            (float&)m_dutycycle_now,
            0.0,
            ramp_step_no_lim * fabsf(current_nofilter - m_conf->lo_current_max)
                * m_conf->m_current_backoff_gain);
        limit_delay = 1;
      }
      else if (current_nofilter < m_conf->lo_current_min) {
        step_towards(
            (float&)m_dutycycle_now,
            m_direction ? m_conf->l_max_duty : -m_conf->l_max_duty,
            ramp_step_no_lim * fabsf(current_nofilter - m_conf->lo_current_min)
                * m_conf->m_current_backoff_gain);
        limit_delay = 1;
      }
      else if (current_in_nofilter > m_conf->lo_in_current_max) {
        step_towards(
            (float&)m_dutycycle_now,
            0.0,
            ramp_step_no_lim
                * fabsf(current_in_nofilter - m_conf->lo_in_current_max)
                * m_conf->m_current_backoff_gain);
        limit_delay = 1;
      }
      else if (current_in_nofilter < m_conf->lo_in_current_min) {
        step_towards(
            (float&)m_dutycycle_now,
            m_direction ? m_conf->l_max_duty : -m_conf->l_max_duty,
            ramp_step_no_lim
                * fabsf(current_in_nofilter - m_conf->lo_in_current_min)
                * m_conf->m_current_backoff_gain);
        limit_delay = 1;
      }

      if (limit_delay > 0) {
        limit_delay--;
      }
      else {
        m_dutycycle_now = dutycycle_now_tmp;
      }

      // When the set duty cycle is in the opposite direction, make sure that the motor
      // starts again after stopping completely
      if (fabsf(m_dutycycle_now) < m_conf->l_min_duty) {
        if (m_dutycycle_set >= m_conf->l_min_duty) {
          m_dutycycle_now = m_conf->l_min_duty;
        }
        else if (m_dutycycle_set <= -m_conf->l_min_duty) {
          m_dutycycle_now = -m_conf->l_min_duty;
        }
      }

      // Don't start in the opposite direction when the RPM is too high even if the current is low enough.
      if (m_dutycycle_now >= m_conf->l_min_duty
          && rpm < -m_conf->l_max_erpm_fbrake) {
        m_dutycycle_now = -m_conf->l_min_duty;
      }
      else if (m_dutycycle_now <= -m_conf->l_min_duty
          && rpm > m_conf->l_max_erpm_fbrake) {
        m_dutycycle_now = m_conf->l_min_duty;
      }

      set_duty_cycle_ll(m_dutycycle_now);
    }

    mc_interface::mc_timer_isr();

    if (encoder::is_configured()) {
      run_pid_control_pos(1.0 / m_switching_frequency_now);
    }

    m_last_adc_isr_duration = TIM12->CNT / TIM12_FREQ;
  }

  void set_detect(void) {
    if (mc_interface::try_input()) {
      return;
    }

    m_control_mode = CONTROL_MODE_NONE;
    stop_pwm_hw();

    set_switching_frequency(m_conf->m_bldc_f_sw_max);

    for (int i = 0; i < 6; i++) {
      detect_currents[i] = 0;
      m_detect_currents_avg[i] = 0;
      m_detect_avg_samples[i] = 0;
    }

    m_state = MC_STATE_DETECTING;
  }

  degree_t get_detect_pos(void) {
    float v[6];
    v[0] = m_detect_currents_avg[0] / m_detect_avg_samples[0];
    v[1] = m_detect_currents_avg[1] / m_detect_avg_samples[1];
    v[2] = m_detect_currents_avg[2] / m_detect_avg_samples[2];
    v[3] = m_detect_currents_avg[3] / m_detect_avg_samples[3];
    v[4] = m_detect_currents_avg[4] / m_detect_avg_samples[4];
    v[5] = m_detect_currents_avg[5] / m_detect_avg_samples[5];

    for (int i = 0; i < 6; i++) {
      m_detect_currents_avg[i] = 0;
      m_detect_avg_samples[i] = 0;
    }

    float v0 = v[0] + v[3];
    float v1 = v[1] + v[4];
    float v2 = v[2] + v[5];

    float offset = (v0 + v1 + v2) / 3.0;
    v0 -= offset;
    v1 -= offset;
    v2 -= offset;

    float amp = sqrtf((v0 * v0 + v1 * v1 + v2 * v2) / 1.5);
    v0 /= amp;
    v1 /= amp;
    v2 /= amp;

    float ph[1];
    ph[0] = asinf(v0) * 180.0 / M_PI;

    float res = ph[0];
    if (v1 < v2) {
      res = 180 - ph[0];
    }

    norm_angle(res);

    return res;
  }

  float read_reset_avg_cycle_integrator(void) {
    float res = m_cycle_integrator_sum / m_cycle_integrator_iterations;
    m_cycle_integrator_sum = 0;
    m_cycle_integrator_iterations = 0;
    return res;
  }

  /**
   * Set the commutation mode for sensorless commutation.
   *
   * @param mode
   * COMM_MODE_INTEGRATE: More robust, but requires many parameters.
   * COMM_MODE_DELAY: Like most hobby ESCs. Requires less parameters,
   * but has worse startup and is less robust.
   *
   */
  void set_comm_mode(mc_comm_mode mode) {
    m_conf->comm_mode = mode;
  }

  mc_comm_mode get_comm_mode(void) {
    return m_conf->comm_mode;
  }

  second_t get_last_adc_isr_duration(void) {
    return m_last_adc_isr_duration;
  }

  second_t get_last_inj_adc_isr_duration(void) {
    return m_last_inj_adc_isr_duration;
  }

  mc_rpm_dep_struct const& get_rpm_dep(void) {
    return rpm_dep;
  }

  bool is_dccal_done(void) {
    return m_dccal_done;
  }

  void switch_comm_mode(mc_comm_mode next) {
    m_comm_mode_next = next;
  }

  /**
   * Reset the hall sensor detection table
   */
  void reset_hall_detect_table(void) {
    memset((void*)m_hall_detect_table, 0,
           sizeof(m_hall_detect_table[0][0]) * 8 * 7);
  }

  /**
   * Get the current detected hall sensor table
   *
   * @param table
   * Pointer to a table where the result should be stored
   *
   * @return
   * 0: OK
   * -1: Invalid hall sensor output
   * -2: WS2811 enabled
   * -3: Encoder enabled
   */
  int get_hall_detect_result(int8_t *table) {
    if (WS2811_ENABLE) {
      return -2;
    }
    else if (m_conf->m_sensor_port_mode != SENSOR_PORT_MODE_HALL) {
      return -3;
    }

    for (int i = 0; i < 8; i++) {
      int samples = 0;
      int res = -1;
      for (int j = 1; j < 7; j++) {
        if (m_hall_detect_table[i][j] > samples) {
          samples = m_hall_detect_table[i][j];
          if (samples > 15) {
            res = j;
          }
        }
        table[i] = res;
      }
    }

    int invalid_samp_num = 0;
    int nums[7] = {0, 0, 0, 0, 0, 0, 0};
    int tot_nums = 0;
    for (int i = 0; i < 8; i++) {
      if (table[i] == -1) {
        invalid_samp_num++;
      }
      else {
        if (!nums[table[i]]) {
          nums[table[i]] = 1;
          tot_nums++;
        }
      }
    }

    if (invalid_samp_num == 2 && tot_nums == 6) {
      return 0;
    }
    else {
      return -1;
    }
  }

  /**
   * Read the current phase of the motor using hall effect sensors
   * @return
   * The phase read.
   */
  int read_hall_phase(void) {
    return m_hall_to_phase_table[read_hall() + (m_direction ? 8 : 0)];
  }

  int read_hall(void) {
    return READ_HALL1() | (READ_HALL2() << 1) | (READ_HALL3() << 2);
  }

  /*
   * Commutation Steps FORWARDS
   * STEP		BR1		BR2		BR3
   * 1		0		+		-
   * 2		+		0		-
   * 3		+		-		0
   * 4		0		-		+
   * 5		-		0		+
   * 6		-		+		0
   *
   * Commutation Steps REVERSE (switch phase 2 and 3)
   * STEP		BR1		BR2		BR3
   * 1		0		-		+
   * 2		+		-		0
   * 3		+		0		-
   * 4		0		+		-
   * 5		-		+		0
   * 6		-		0		+
   */

  void update_adc_sample_pos(MCTimer *timer_tmp) {
    volatile uint32_t duty = timer_tmp->duty;
    volatile uint32_t top = timer_tmp->top;
    volatile uint32_t val_sample = timer_tmp->val_sample;
    volatile uint32_t curr1_sample = timer_tmp->curr1_sample;
    volatile uint32_t curr2_sample = timer_tmp->curr2_sample;

#ifdef HW_HAS_3_SHUNTS
    volatile uint32_t curr3_sample = timer_tmp->curr3_sample;
#endif

    if (duty > (uint32_t)((float)top * m_conf->l_max_duty)) {
      duty = (uint32_t)((float)top * m_conf->l_max_duty);
    }

    m_use_curr_samp_volt = 0;

    // Sample the ADC at an appropriate time during the pwm cycle
    if (is_detecting()) {
      // Voltage samples
      val_sample = duty / 2;

      // Current samples
      curr1_sample = (top - duty) / 2 + duty;
      curr2_sample = (top - duty) / 2 + duty;
#ifdef HW_HAS_3_SHUNTS
      curr3_sample = (top - duty) / 2 + duty;
#endif
    }
    else {
      if (m_conf->pwm_mode == PWM_MODE_BIPOLAR) {
        uint32_t samp_neg = top - 2;
        uint32_t samp_pos = duty + (top - duty) / 2;
        uint32_t samp_zero = top - 2;

        // Voltage and other sampling
        val_sample = top / 4;

        // Current sampling
        // TODO: Adapt for 3 shunts
#ifdef HW_HAS_3_SHUNTS
        curr3_sample = samp_zero;
#endif

        switch (m_comm_step) {
        case 1:
          if (m_direction) {
            curr1_sample = samp_zero;
            curr2_sample = samp_neg;
            m_use_curr_samp_volt = (1 << 1);
          }
          else {
            curr1_sample = samp_zero;
            curr2_sample = samp_pos;
          }
          break;

        case 2:
          if (m_direction) {
            curr1_sample = samp_pos;
            curr2_sample = samp_neg;
            m_use_curr_samp_volt = (1 << 1);
          }
          else {
            curr1_sample = samp_pos;
            curr2_sample = samp_zero;
          }
          break;

        case 3:
          if (m_direction) {
            curr1_sample = samp_pos;
            curr2_sample = samp_zero;
          }
          else {
            curr1_sample = samp_pos;
            curr2_sample = samp_neg;
            m_use_curr_samp_volt = (1 << 1);
          }
          break;

        case 4:
          if (m_direction) {
            curr1_sample = samp_zero;
            curr2_sample = samp_pos;
          }
          else {
            curr1_sample = samp_zero;
            curr2_sample = samp_neg;
            m_use_curr_samp_volt = (1 << 1);
          }
          break;

        case 5:
          if (m_direction) {
            curr1_sample = samp_neg;
            curr2_sample = samp_pos;
            m_use_curr_samp_volt = (1 << 0);
          }
          else {
            curr1_sample = samp_neg;
            curr2_sample = samp_zero;
            m_use_curr_samp_volt = (1 << 0);
          }
          break;

        case 6:
          if (m_direction) {
            curr1_sample = samp_neg;
            curr2_sample = samp_zero;
            m_use_curr_samp_volt = (1 << 0);
          }
          else {
            curr1_sample = samp_neg;
            curr2_sample = samp_pos;
            m_use_curr_samp_volt = (1 << 0);
          }
          break;
        }
      }
      else { // PWM_MODE_SYNCHRONOUS
        // Voltage samples
        val_sample = duty / 2;

        // Current samples
        curr1_sample = duty + (top - duty) / 2;
        if (curr1_sample > (top - 70)) {
          curr1_sample = top - 70;
        }

        curr2_sample = curr1_sample;
#ifdef HW_HAS_3_SHUNTS
        curr3_sample = curr1_sample;
#endif

        // The off sampling time is short, so use the on sampling time
        // where possible
        if (duty > (top / 2)) {
#if CURR1_DOUBLE_SAMPLE
          if (m_comm_step == 2 || m_comm_step == 3) {
            curr1_sample = duty + 90;
            curr2_sample = top - 230;
          }
#endif

#if CURR2_DOUBLE_SAMPLE
          if (m_direction) {
            if (m_comm_step == 4 || m_comm_step == 5) {
              curr1_sample = duty + 90;
              curr2_sample = top - 230;
            }
          }
          else {
            if (m_comm_step == 1 || m_comm_step == 6) {
              curr1_sample = duty + 90;
              curr2_sample = top - 230;
            }
          }
#endif

#ifdef HW_HAS_3_SHUNTS
          if (m_direction) {
            switch (m_comm_step) {
            case 1:
              m_use_curr_samp_volt = (1 << 0) || (1 << 2);
              break;
            case 2:
              m_use_curr_samp_volt = (1 << 1) || (1 << 2);
              break;
            case 3:
              m_use_curr_samp_volt = (1 << 1) || (1 << 2);
              break;
            case 4:
              m_use_curr_samp_volt = (1 << 0) || (1 << 1);
              break;
            case 5:
              m_use_curr_samp_volt = (1 << 0) || (1 << 1);
              break;
            case 6:
              m_use_curr_samp_volt = (1 << 0) || (1 << 2);
              break;
            default:
              break;
            }
          }
          else {
            switch (m_comm_step) {
            case 1:
              m_use_curr_samp_volt = (1 << 0) || (1 << 1);
              break;
            case 2:
              m_use_curr_samp_volt = (1 << 1) || (1 << 2);
              break;
            case 3:
              m_use_curr_samp_volt = (1 << 1) || (1 << 2);
              break;
            case 4:
              m_use_curr_samp_volt = (1 << 0) || (1 << 2);
              break;
            case 5:
              m_use_curr_samp_volt = (1 << 0) || (1 << 2);
              break;
            case 6:
              m_use_curr_samp_volt = (1 << 0) || (1 << 1);
              break;
            default:
              break;
            }
          }
#else
          if (m_direction) {
            switch (m_comm_step) {
              case 1: m_use_curr_samp_volt = (1 << 0) || (1 << 1); break;
              case 2: m_use_curr_samp_volt = (1 << 1); break;
              case 3: m_use_curr_samp_volt = (1 << 1); break;
              case 4: m_use_curr_samp_volt = (1 << 0); break;
              case 5: m_use_curr_samp_volt = (1 << 0); break;
              case 6: m_use_curr_samp_volt = (1 << 0) || (1 << 1); break;
              default: break;
            }
          }
          else {
            switch (m_comm_step) {
              case 1: m_use_curr_samp_volt = (1 << 0); break;
              case 2: m_use_curr_samp_volt = (1 << 1); break;
              case 3: m_use_curr_samp_volt = (1 << 1); break;
              case 4: m_use_curr_samp_volt = (1 << 0) || (1 << 1); break;
              case 5: m_use_curr_samp_volt = (1 << 0) || (1 << 1); break;
              case 6: m_use_curr_samp_volt = (1 << 0); break;
              default: break;
            }
          }
#endif
        }
      }
    }

    timer_tmp->val_sample = val_sample;
    timer_tmp->curr1_sample = curr1_sample;
    timer_tmp->curr2_sample = curr2_sample;
#ifdef HW_HAS_3_SHUNTS
    timer_tmp->curr3_sample = curr3_sample;
#endif
  }

  void update_rpm_tacho(void) {
    int step = m_comm_step - 1;
    static int last_step = 0;
    int tacho_diff = (step - last_step) % 6;
    last_step = step;

    if (tacho_diff > 3) {
      tacho_diff -= 6;
    }
    else if (tacho_diff < -2) {
      tacho_diff += 6;
    }

    if (tacho_diff != 0) {
      rpm_dep.comms += tacho_diff;
      rpm_dep.time_at_comm += TIM2->CNT;
      TIM2->CNT = 0;
    }

    // Tachometers
    m_tachometer_for_direction += tacho_diff;
    m_tachometer_abs += tacho_diff;

    if (m_direction) {
      m_tachometer += tacho_diff;
    }
    else {
      m_tachometer -= tacho_diff;
    }
  }

  void update_sensor_mode(void) {
    if (m_conf->sensor_mode == SENSOR_MODE_SENSORLESS
        || (m_conf->sensor_mode == SENSOR_MODE_HYBRID
            && fabsf(get_rpm()) > m_conf->hall_sl_erpm)) {
      m_sensorless_now = true;
    }
    else {
      m_sensorless_now = false;
    }
  }
  /*
   * @steps: how many steps to commutate forward. Typically 1 or, if motor is stuck, 2.
   */
  void commutate(int steps) {
    m_last_pwm_cycles_sum = m_pwm_cycles_sum;
    m_last_pwm_cycles_sums[m_comm_step - 1] = m_pwm_cycles_sum;
    m_pwm_cycles_sum = 0;
    m_pwm_cycles = 0;

    if (m_sensorless_now) {
      m_comm_step += steps;
      while (m_comm_step > 6) {
        m_comm_step -= 6;
      }
      while (m_comm_step < 1) {
        m_comm_step += 6;
      }

      update_rpm_tacho();

      if (!(m_state == MC_STATE_RUNNING)) {
        update_sensor_mode();
        return;
      }

      set_next_comm_step(m_comm_step);
    }

    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    m_has_commutated = 1;

    MCTimer timer_tmp;

    sys_lock_cnt();
    timer_tmp = m_timer_struct;
    sys_unlock_cnt();

    update_adc_sample_pos(&timer_tmp);
    set_next_timer_settings(&timer_tmp);
    update_sensor_mode();
    m_conf->comm_mode = m_comm_mode_next;
  }

  void set_next_timer_settings(MCTimer const*settings) {
    sys_lock_cnt();
    m_timer_struct = *settings;
    m_timer_struct.updated = false;
    sys_unlock_cnt();

    update_timer_attempt();
  }

  /**
   * Try to apply the new timer settings. This is really not an elegant solution, but for now it is
   * the best I can come up with.
   */
  void update_timer_attempt(void) {
    sys_lock_cnt();

    // Set the next timer settings if an update is far enough away
    if (!m_timer_struct.updated && TIM1->CNT > 10
        && TIM1->CNT < (TIM1->ARR - 500)) {
      // Disable preload register updates
      TIM1->CR1 |= TIM_CR1_UDIS;
      TIM8->CR1 |= TIM_CR1_UDIS;

      // Set the new configuration
      TIM1->ARR = m_timer_struct.top;
      TIM1->CCR1 = m_timer_struct.duty;
      TIM1->CCR2 = m_timer_struct.duty;
      TIM1->CCR3 = m_timer_struct.duty;
      TIM8->CCR1 = m_timer_struct.val_sample;
      TIM1->CCR4 = m_timer_struct.curr1_sample;
      TIM8->CCR2 = m_timer_struct.curr2_sample;
#ifdef HW_HAS_3_SHUNTS
      TIM8->CCR3 = m_timer_struct.curr3_sample;
#endif

      // Enables preload register updates
      TIM1->CR1 &= ~TIM_CR1_UDIS;
      TIM8->CR1 &= ~TIM_CR1_UDIS;
      m_timer_struct.updated = true;
    }

    sys_unlock_cnt();
  }

  void set_switching_frequency(hertz_t frequency) {
    m_switching_frequency_now = frequency;
    MCTimer timer_tmp;

    sys_lock_cnt();
    timer_tmp = m_timer_struct;
    sys_unlock_cnt();

    timer_tmp.top = (int)static_cast<float>(SYSTEM_CORE_CLOCK / m_switching_frequency_now);
    update_adc_sample_pos(&timer_tmp);
    set_next_timer_settings(&timer_tmp);
  }

  /**
   * @next_step: step number in [1,6] to move to
   */
  void set_next_comm_step(int next_step) {

    uint16_t positive_oc_mode = TIM_OCMode_PWM1;
    uint16_t negative_oc_mode = TIM_OCMode_Inactive;

    uint16_t positive_highside = TIM_CCx_Enable;
    uint16_t positive_lowside = TIM_CCxN_Enable;

    uint16_t negative_highside = TIM_CCx_Enable;
    uint16_t negative_lowside = TIM_CCxN_Enable;

    if (!is_detecting()) {
      switch (m_conf->pwm_mode) {
      case PWM_MODE_NONSYNCHRONOUS_HISW:
        positive_lowside = TIM_CCxN_Disable;
        break;

      case PWM_MODE_SYNCHRONOUS:
        break;

      case PWM_MODE_BIPOLAR:
        negative_oc_mode = TIM_OCMode_PWM2;
        break;
      }
    }

    if (next_step == 1) {
      if (m_direction) {
#ifdef HW_HAS_DRV8313
        DISABLE_BR1()
        ;
        ENABLE_BR2()
        ;
        ENABLE_BR3()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_2, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_2, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_3, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_3, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, negative_lowside);
      }
      else {
#ifdef HW_HAS_DRV8313
        DISABLE_BR1()
        ;
        ENABLE_BR3()
        ;
        ENABLE_BR2()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_3, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_3, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_2, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_2, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, negative_lowside);
      }
    }
    else if (next_step == 2) {
      if (m_direction) {
#ifdef HW_HAS_DRV8313
        DISABLE_BR2()
        ;
        ENABLE_BR1()
        ;
        ENABLE_BR3()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_1, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_1, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_3, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_3, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, negative_lowside);
      }
      else {
#ifdef HW_HAS_DRV8313
        DISABLE_BR3()
        ;
        ENABLE_BR1()
        ;
        ENABLE_BR2()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_1, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_1, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_2, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_2, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, negative_lowside);
      }
    }
    else if (next_step == 3) {
      if (m_direction) {
#ifdef HW_HAS_DRV8313
        DISABLE_BR3()
        ;
        ENABLE_BR1()
        ;
        ENABLE_BR2()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_1, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_1, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_2, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_2, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, negative_lowside);
      }
      else {
#ifdef HW_HAS_DRV8313
        DISABLE_BR2()
        ;
        ENABLE_BR1()
        ;
        ENABLE_BR3()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_1, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_1, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_3, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_3, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, negative_lowside);
      }
    }
    else if (next_step == 4) {
      if (m_direction) {
#ifdef HW_HAS_DRV8313
        DISABLE_BR1()
        ;
        ENABLE_BR3()
        ;
        ENABLE_BR2()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_3, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_3, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_2, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_2, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, negative_lowside);
      }
      else {
#ifdef HW_HAS_DRV8313
        DISABLE_BR1()
        ;
        ENABLE_BR2()
        ;
        ENABLE_BR3()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_2, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_2, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_3, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_3, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, negative_lowside);
      }
    }
    else if (next_step == 5) {
      if (m_direction) {
#ifdef HW_HAS_DRV8313
        DISABLE_BR2()
        ;
        ENABLE_BR3()
        ;
        ENABLE_BR1()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_3, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_3, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_1, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_1, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, negative_lowside);
      }
      else {
#ifdef HW_HAS_DRV8313
        DISABLE_BR3()
        ;
        ENABLE_BR2()
        ;
        ENABLE_BR1()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_2, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_2, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_1, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_1, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, negative_lowside);
      }
    }
    else if (next_step == 6) {
      if (m_direction) {
#ifdef HW_HAS_DRV8313
        DISABLE_BR3()
        ;
        ENABLE_BR2()
        ;
        ENABLE_BR1()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_2, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_2, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_1, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_1, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, negative_lowside);
      }
      else {
#ifdef HW_HAS_DRV8313
        DISABLE_BR2()
        ;
        ENABLE_BR3()
        ;
        ENABLE_BR1()
        ;
#endif
        // 0
        TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_3, positive_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_3, positive_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, positive_lowside);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_1, negative_oc_mode);
        TIM_CCxCmd(TIM1, TIM_Channel_1, negative_highside);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, negative_lowside);
      }
    }
    else {
#ifdef HW_HAS_DRV8313
      DISABLE_BR1()
      ;
      DISABLE_BR2()
      ;
      DISABLE_BR3()
      ;
#endif
      // Invalid phase.. stop PWM!
      TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
      TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
      TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

      TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
      TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
      TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

      TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
      TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
      TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
    }
  }
} //namespace pwm
