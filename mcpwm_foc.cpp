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

#include "mcpwm_foc.h"
#include "mc_interface.h"
#include "ch.h"
#include "hal.h"
#include "digital_filter.h"
#include "utils.h"
#include "ledpwm.h"
#include "terminal.h"
#include "encoder.h"
#include "commands.h"
#include "timeout.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

using namespace utils;
namespace{
  inline void sincos_rad(radian_t angle, float* sin, float* cos){
    sincosf(static_cast<float>(angle), sin, cos);
  }
}

namespace mcpwm_foc{
  // Private types
  struct motor_state_t{
      ampere_t id_target;
      ampere_t iq_target;
      float max_duty;
      float duty_now;
      radian_t phase;
      ampere_t i_alpha;
      ampere_t i_beta;
      ampere_t i_abs;
      ampere_t i_abs_filter;
      ampere_t i_bus;
      volt_t v_bus;
      volt_t v_alpha;
      volt_t v_beta;
      float mod_d;
      float mod_q;
      ampere_t id;
      ampere_t iq;
      ampere_t id_filter;
      ampere_t iq_filter;
      volt_t vd;
      volt_t vq;
      volt_t vd_int;
      volt_t vq_int;
      uint32_t svm_sector;
  };

  struct mc_sample_t{
      volatile size_t sample_num = 0;
      volatil_ ampere_t avg_current_tot = 0_A;
      volatil_ volt_t avg_voltage_tot = 0.0_V;
      volatile bool measure_inductance_now = false;
      volatile dutycycle_t measure_inductance_duty = 0.0;

      void reset() {
        sample_num = 0;
        avg_current_tot = 0_A;
        avg_voltage_tot = 0.0_V;
      }

      ampere_t get_avg_current() const {
        return avg_current_tot / sample_num;
      }

      volt_t get_avg_voltage() const {
        return avg_voltage_tot / sample_num;
      }
  };

  // Private variables
  volatil_ mc_configuration *m_conf;
  volatile mc_state m_state;
  volatile mc_control_mode m_control_mode;
  volatil_ motor_state_t m_motor_state;
  volatile int m_curr0_sum;
  volatile int m_curr1_sum;
  volatile int m_curr2_sum;
  volatile int m_curr_samples;
  volatile int m_curr0_offset;
  volatile int m_curr1_offset;
  volatile int m_curr2_offset;
  volatile bool m_phase_override;
  volatil_ radian_t m_phase_now_override;
  volatile float m_duty_cycle_set;
  volatil_ ampere_t m_id_set;
  volatil_ ampere_t m_iq_set;
  volatil_ radians_per_second_t m_openloop_speed;
  volatile bool m_dccal_done;
  volatile bool m_output_on;
  volatil_ degree_t m_pos_pid_set;
  volatil_ rpm_t m_speed_pid_set_rpm;
  volatil_ radian_t m_phase_now_observer;
  volatil_ radian_t m_phase_now_observer_override;
  volatile bool m_phase_observer_override;
  volatil_ radian_t m_phase_now_encoder;
  volatil_ radian_t m_phase_now_encoder_no_index;
  volatil_ weber_t m_observer_x1;
  volatil_ weber_t m_observer_x2;
  volatil_ radian_t m_pll_phase;
  volatil_ radians_per_second_t m_pll_speed;
  mc_sample_t m_samples;
  volatile int m_tachometer;
  volatile int m_tachometer_abs;
  volatil_ second_t m_last_adc_isr_duration;
  volatil_ degree_t m_pos_pid_now;
  volatile bool m_init_done;
  volatil_ decltype(1_Hz/(1_Wb*1_Wb)) m_gamma_now;

  // Private functions
  void do_dc_cal(void);
  void observer_update(volt_t v_alpha, volt_t v_beta, ampere_t i_alpha, ampere_t i_beta,
          second_t dt, volatil_ weber_t& x1, volatil_ weber_t& x2, volatil_ radian_t& phase);
  void pll_run(radian_t phase,
               second_t dt,
               volatil_ radian_t& phase_var,
               volatil_ radians_per_second_t& speed_var);
  void control_current(volatil_ motor_state_t& state_m, second_t dt);
  void svm(float alpha, float beta, uint32_t PWMHalfPeriod,
          uint32_t& tAout, uint32_t& tBout, uint32_t& tCout, uint32_t& svm_sector);
  void run_pid_control_pos(degree_t angle_now, degree_t angle_set, second_t dt);
  void run_pid_control_speed(second_t dt);
  void run_foc_control(second_t dt);
  void read_current_measures(bool is_v7);
  void track_bemf(second_t dt);
  void stop_pwm_hw(void);
  void start_pwm_hw(void);
  int read_hall(void);
  radian_t correct_encoder(radian_t obs_angle, radian_t enc_angle, radians_per_second_t speed);
  radian_t correct_hall(radian_t angle, radians_per_second_t speed, second_t dt);

  // Threads
  THD_WORKING_AREA(timer_thread_wa, 2048);
  THD_FUNCTION(timer_thread, arg);
  volatile bool m_timer_thd_stop;


  class PhaseOverride{
    mc_interface::Lock _mc_interface_lock;
    timeout::Disabler _timeout_disabler;

  public:
    PhaseOverride(ampere_t id, ampere_t iq){
      m_phase_override = true;
      m_phase_now_override = 0.0_rad;
      m_id_set = id;
      m_iq_set = iq;
      m_control_mode = CONTROL_MODE_CURRENT;
      m_state = MC_STATE_RUNNING;
    }
    ~PhaseOverride(){
      // undo locked settings and stop motor
      m_id_set = 0_A;
      m_iq_set = 0_A;
      m_phase_override = false;
      m_control_mode = CONTROL_MODE_NONE;
      m_state = MC_STATE_OFF;
      stop_pwm_hw();
    }
  };

  // Macros
#ifdef HW_HAS_3_SHUNTS
  #define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
          TIM1->CR1 |= TIM_CR1_UDIS; \
          TIM1->CCR1 = duty1; \
          TIM1->CCR2 = duty2; \
          TIM1->CCR3 = duty3; \
          TIM1->CR1 &= ~TIM_CR1_UDIS;
#else
  #define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
          TIM1->CR1 |= TIM_CR1_UDIS; \
          TIM1->CCR1 = duty1; \
          TIM1->CCR2 = duty3; \
          TIM1->CCR3 = duty2; \
          TIM1->CR1 &= ~TIM_CR1_UDIS;
#endif

  #define TIMER_UPDATE_SAMP(samp) \
          TIM8->CCR1 = samp;

  #define TIMER_UPDATE_SAMP_TOP(samp, top) \
          TIM1->CR1 |= TIM_CR1_UDIS; \
          TIM8->CR1 |= TIM_CR1_UDIS; \
          TIM1->ARR = top; \
          TIM8->CCR1 = samp; \
          TIM1->CR1 &= ~TIM_CR1_UDIS; \
          TIM8->CR1 &= ~TIM_CR1_UDIS;

#ifdef HW_HAS_3_SHUNTS
  #define TIMER_UPDATE_DUTY_SAMP(duty1, duty2, duty3, samp) \
          TIM1->CR1 |= TIM_CR1_UDIS; \
          TIM8->CR1 |= TIM_CR1_UDIS; \
          TIM1->CCR1 = duty1; \
          TIM1->CCR2 = duty2; \
          TIM1->CCR3 = duty3; \
          TIM8->CCR1 = samp; \
          TIM1->CR1 &= ~TIM_CR1_UDIS; \
          TIM8->CR1 &= ~TIM_CR1_UDIS;
#else
  #define TIMER_UPDATE_DUTY_SAMP(duty1, duty2, duty3, samp) \
          TIM1->CR1 |= TIM_CR1_UDIS; \
          TIM8->CR1 |= TIM_CR1_UDIS; \
          TIM1->CCR1 = duty1; \
          TIM1->CCR2 = duty3; \
          TIM1->CCR3 = duty2; \
          TIM8->CCR1 = samp; \
          TIM1->CR1 &= ~TIM_CR1_UDIS; \
          TIM8->CR1 &= ~TIM_CR1_UDIS;
#endif

  void init(mc_configuration *configuration) {
      sys_lock_cnt();

      m_init_done = false;

      TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
      TIM_OCInitTypeDef  TIM_OCInitStructure;
      TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

      // Initialize variables
      m_conf = configuration;
      m_state = MC_STATE_OFF;
      m_control_mode = CONTROL_MODE_NONE;
      m_curr0_sum = 0;
      m_curr1_sum = 0;
      m_curr2_sum = 0;
      m_curr_samples = 0;
      m_dccal_done = false;
      m_phase_override = false;
      m_phase_now_override = 0.0_rad;
      m_duty_cycle_set = 0.0;
      m_id_set = 0_A;
      m_iq_set = 0_A;
      m_openloop_speed = 0.0_rad_per_s;
      m_output_on = false;
      m_pos_pid_set = 0_deg;
      m_speed_pid_set_rpm = 0_rpm;
      m_phase_now_observer = 0.0_rad;
      m_phase_now_observer_override = 0.0_rad;
      m_phase_observer_override = false;
      m_phase_now_encoder = 0.0_rad;
      m_phase_now_encoder_no_index = 0.0_rad;
      m_observer_x1 = 0.0_Wb;
      m_observer_x2 = 0.0_Wb;
      m_pll_phase = 0.0_rad;
      m_pll_speed = 0.0_rad_per_s;
      m_tachometer = 0;
      m_tachometer_abs = 0;
      m_last_adc_isr_duration = 0_s;
      m_pos_pid_now = 0_deg;
      m_gamma_now = decltype(m_gamma_now){0.0};
      memset((void*)&m_motor_state, 0, sizeof(motor_state_t));
      memset((void*)&m_samples, 0, sizeof(mc_sample_t));

      TIM_DeInit(TIM1);
      TIM_DeInit(TIM8);
      TIM1->CNT = 0;
      TIM8->CNT = 0;

      // TIM1 clock enable
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

      // Time Base configuration
      TIM_TimeBaseStructure.TIM_Prescaler = 0;
      TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
      TIM_TimeBaseStructure.TIM_Period = hw::SYSTEM_CORE_CLOCK / m_conf->foc_f_sw;
      TIM_TimeBaseStructure.TIM_ClockDivision = 0;
      TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

      TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

      // Channel 1, 2 and 3 Configuration in PWM mode
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
      TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;
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
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

      dmaStreamAllocate(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)),
              3,
              (stm32_dmaisr_t)adc_interrupt_handler,
              (void *)0);

      // DMA for the ADC
      DMA_InitStructure.DMA_Channel = DMA_Channel_0;
      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value;
      DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
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
      // Note that the ADC is running at 42MHz, which is higher than the
      // specified 36MHz in the data sheet, but it works.
      ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
      ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
      ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
      ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
      ADC_CommonInit(&ADC_CommonInitStructure);

      // Channel-specific settings
      ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
      ADC_InitStructure.ADC_ScanConvMode = ENABLE;
      ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
      ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
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

      hw::setup_adc_channels();

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

      // ADC sampling locations
      stop_pwm_hw();

      // Sample intervals. For now they are fixed with voltage samples in the center of V7
      // and current samples in the center of V0
      TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);

      // Enable CC1 interrupt, which will be fired in V0 and V7
      TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);
      nvicEnableVector(TIM8_CC_IRQn, 6);

      sys_unlock_cnt();

      // Calibrate current offset
      ENABLE_GATE();
      DCCAL_OFF();
      do_dc_cal();

      // Various time measurements
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

      // Time base configuration
      TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
      TIM_TimeBaseStructure.TIM_Prescaler = (hw::TIM12_CLOCK / TIM12_FREQ) - 1;
      TIM_TimeBaseStructure.TIM_ClockDivision = 0;
      TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
      TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

      TIM_Cmd(TIM12, ENABLE);

      // Start threads
      m_timer_thd_stop = false;
      chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);

      // WWDG configuration
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
      WWDG_SetPrescaler(WWDG_Prescaler_1);
      WWDG_SetWindowValue(255);
      WWDG_Enable(100);

      m_init_done = true;
  }

  void deinit(void) {
      m_init_done = false;

      WWDG_DeInit();

      m_timer_thd_stop = true;

      while (m_timer_thd_stop) {
          chThdSleepMilliseconds(1);
      }

      TIM_DeInit(TIM1);
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
      m_conf = configuration;

      m_control_mode = CONTROL_MODE_NONE;
      m_state = MC_STATE_OFF;
      stop_pwm_hw();
      uint32_t top = hw::SYSTEM_CORE_CLOCK / m_conf->foc_f_sw;
      TIMER_UPDATE_SAMP_TOP(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);
  }

  mc_state get_state(void) {
      return m_state;
  }

  bool is_dccal_done(void) {
      return m_dccal_done;
  }

  /**
   * Switch off all FETs.TIM12->CNT = 0;
   */
  void stop_pwm(void) {
      set_current(0_A);
  }

  /**
   * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
   * stop the motor.
   *
   * @param dutyCycle
   * The duty cycle to use.
   */
  void set_duty(float dutyCycle) {
      m_control_mode = CONTROL_MODE_DUTY;
      m_duty_cycle_set = dutyCycle;

      if (m_state != MC_STATE_RUNNING) {
          m_state = MC_STATE_RUNNING;
      }
  }

  /**
   * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
   * stop the motor.
   *
   * WARNING: This function does not use ramping. A too large step with a large motor
   * can destroy hardware.
   *
   * @param dutyCycle
   * The duty cycle to use.
   */
  void set_duty_noramp(float dutyCycle) {
      // TODO: Actually do this without ramping
      set_duty(dutyCycle);
  }

  /**
   * Use PID rpm control. Note that this value has to be multiplied by half of
   * the number of motor poles.
   *
   * @param rpm
   * The electrical RPM goal value to use.
   */
  void set_pid_speed(rpm_t rpm) {
      m_control_mode = CONTROL_MODE_SPEED;
      m_speed_pid_set_rpm = rpm;

      if (m_state != MC_STATE_RUNNING) {
          m_state = MC_STATE_RUNNING;
      }
  }

  /**
   * Use PID position control. Note that this only works when encoder support
   * is enabled.
   *
   * @param pos
   * The desired position of the motor in degrees.
   */
  void set_pid_pos(degree_t pos) {
      m_control_mode = CONTROL_MODE_POS;
      m_pos_pid_set = pos;

      if (m_state != MC_STATE_RUNNING) {
          m_state = MC_STATE_RUNNING;
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
          m_state = MC_STATE_OFF;
          stop_pwm_hw();
          return;
      }

      m_control_mode = CONTROL_MODE_CURRENT;
      m_iq_set = current;

      if (m_state != MC_STATE_RUNNING) {
          m_state = MC_STATE_RUNNING;
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
          m_state = MC_STATE_OFF;
          stop_pwm_hw();
          return;
      }

      m_control_mode = CONTROL_MODE_CURRENT_BRAKE;
      m_iq_set = current;

      if (m_state != MC_STATE_RUNNING) {
          m_state = MC_STATE_RUNNING;
      }
  }

  /**
   * Apply a fixed static current vector in open loop to emulate an electric
   * handbrake.
   *
   * @param current
   * The brake current to use.
   */
  void set_handbrake(ampere_t current) {
      if (fabsf(current) < m_conf->cc_min_current) {
          m_control_mode = CONTROL_MODE_NONE;
          m_state = MC_STATE_OFF;
          stop_pwm_hw();
          return;
      }

      m_control_mode = CONTROL_MODE_HANDBRAKE;
      m_iq_set = current;

      if (m_state != MC_STATE_RUNNING) {
          m_state = MC_STATE_RUNNING;
      }
  }

  /**
   * Produce an openloop rotating current.
   *
   * @param current
   * The current to use.
   *
   * @param rpm
   * The RPM to use.
   */
  void set_openloop(ampere_t current, rpm_t rpm) {
      if (fabsf(current) < m_conf->cc_min_current) {
          m_control_mode = CONTROL_MODE_NONE;
          m_state = MC_STATE_OFF;
          stop_pwm_hw();
          return;
      }

      truncate_number_abs(current, m_conf->l_current_max);

      m_control_mode = CONTROL_MODE_OPENLOOP;
      m_iq_set = current;
#ifdef USE_UNITS
      m_openloop_speed = rpm;
#else
      m_openloop_speed = rpm * ((2.0 * M_PI) / 60.0);
#endif

      if (m_state != MC_STATE_RUNNING) {
          m_state = MC_STATE_RUNNING;
      }
  }

  float get_duty_cycle_set(void) {
      return m_duty_cycle_set;
  }

  float get_duty_cycle_now(void) {
      return m_motor_state.duty_now;
  }

  degree_t get_pid_pos_set(void) {
      return m_pos_pid_set;
  }

  degree_t get_pid_pos_now(void) {
      return m_pos_pid_now;
  }

  /**
   * Get the current switching frequency.
   *
   * @return
   * The switching frequency in Hz.
   */
  hertz_t get_switching_frequency_now(void) {
      return m_conf->foc_f_sw;
  }

  /**
   * Get the current sampling frequency.
   *
   * @return
   * The sampling frequency in Hz.
   */
  hertz_t get_sampling_frequency_now(void) {
  #ifdef HW_HAS_PHASE_SHUNTS
      if (m_conf->foc_sample_v0_v7) {
          return m_conf->foc_f_sw;
      } else {
          return m_conf->foc_f_sw / 2.0;
      }
  #else
      return m_conf->foc_f_sw / 2.0;
  #endif
  }

  /**
   * Calculate the current RPM of the motor. This is a signed value and the sign
   * depends on the direction the motor is rotating in. Note that this value has
   * to be divided by half the number of motor poles.
   *
   * @return
   * The RPM value.
   */
  rpm_t get_rpm(void) {
#ifdef USE_UNITS
      return m_pll_speed;
#else
      return m_pll_speed / ((2.0 * M_PI) / 60.0);
#endif
  }

  /**
   * Get the motor current. The sign of this value will
   * represent whether the motor is drawing (positive) or generating
   * (negative) current. This is the q-axis current which produces torque.
   *
   * @return
   * The motor current.
   */
  ampere_t get_tot_current(void) {
      return SIGN(m_motor_state.vq) * m_motor_state.iq;
  }

  /**
   * Get the filtered motor current. The sign of this value will
   * represent whether the motor is drawing (positive) or generating
   * (negative) current. This is the q-axis current which produces torque.
   *
   * @return
   * The filtered motor current.
   */
  ampere_t get_tot_current_filtered(void) {
      return SIGN(m_motor_state.vq) * m_motor_state.iq_filter;
  }

  /**
   * Get the magnitude of the motor current, which includes both the
   * D and Q axis.
   *
   * @return
   * The magnitude of the motor current.
   */
  ampere_t get_abs_motor_current(void) {
      return m_motor_state.i_abs;
  }

  /**
   * Get the magnitude of the motor voltage.
   *
   * @return
   * The magnitude of the motor voltage.
   */
  volt_t get_abs_motor_voltage(void) {
      const auto vd_tmp = m_motor_state.vd;
      const auto vq_tmp = m_motor_state.vq;
      return units::math::sqrt(SQ(vd_tmp) + SQ(vq_tmp));
  }

  /**
   * Get the filtered magnitude of the motor current, which includes both the
   * D and Q axis.
   *
   * @return
   * The magnitude of the motor current.
   */
  ampere_t get_abs_motor_current_filtered(void) {
      return m_motor_state.i_abs_filter;
  }

  /**
   * Get the motor current. The sign of this value represents the direction
   * in which the motor generates torque.
   *
   * @return
   * The motor current.
   */
  ampere_t get_tot_current_directional(void) {
      return m_motor_state.iq;
  }

  /**
   * Get the filtered motor current. The sign of this value represents the
   * direction in which the motor generates torque.
   *
   * @return
   * The filtered motor current.
   */
  ampere_t get_tot_current_directional_filtered(void) {
      return m_motor_state.iq_filter;
  }

  /**
   * Get the direct axis motor current.
   *
   * @return
   * The D axis current.
   */
  ampere_t get_id(void) {
      return m_motor_state.id;
  }

  /**
   * Get the quadrature axis motor current.
   *
   * @return
   * The Q axis current.
   */
  ampere_t get_iq(void) {
      return m_motor_state.iq;
  }

  /**
   * Get the input current to the motor controller.
   *
   * @return
   * The input current.
   */
  ampere_t get_tot_current_in(void) {
      return m_motor_state.i_bus;
  }

  /**
   * Get the filtered input current to the motor controller.
   *
   * @return
   * The filtered input current.
   */
  ampere_t get_tot_current_in_filtered(void) {
      return m_motor_state.i_bus; // TODO: Calculate filtered current?
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
   * Read the motor phase.
   *
   * @return
   * The phase angle in degrees.
   */
  degree_t get_phase(void) {
#ifdef USE_UNITS
      degree_t angle = m_motor_state.phase;
#else
      degree_t angle = m_motor_state.phase * (180.0 / M_PI);
#endif
      norm_angle(angle);
      return angle;
  }

  /**
   * Read the phase that the observer has calculated.
   *
   * @return
   * The phase angle in degrees.
   */
  degree_t get_phase_observer(void) {
#ifdef USE_UNITS
      degree_t angle = m_phase_now_observer;
#else
      degree_t angle = m_phase_now_observer * (180.0 / M_PI);
#endif
      norm_angle(angle);
      return angle;
  }

  /**
   * Read the phase from based on the encoder.
   *
   * @return
   * The phase angle in degrees.
   */
  degree_t get_phase_encoder(void) {
#ifdef USE_UNITS
      degree_t angle = m_phase_now_encoder;
#else
      degree_t angle = m_phase_now_encoder * (180.0 / M_PI);
#endif
      norm_angle(angle);
      return angle;
  }

  volt_t get_vd(void) {
      return m_motor_state.vd;
  }

  volt_t get_vq(void) {
      return m_motor_state.vq;
  }

  /**
   * Measure encoder offset and direction.
   *
   * @param current
   * The locking open loop current for the motor.
   *
   * @param offset
   * The detected offset.
   *
   * @param ratio
   * The ratio between electrical and mechanical revolutions
   *
   * @param direction
   * The detected direction.
   */
  void encoder_detect(ampere_t current, bool print, degree_t &offset, float &ratio, bool &inverted) {

      PhaseOverride lockedCurrentContext(current, 0_A);

      // Save configuration
      auto offset_old = m_conf->foc_encoder_offset;
      auto inverted_old = m_conf->foc_encoder_inverted;
      auto ratio_old = m_conf->foc_encoder_ratio;

      m_conf->foc_encoder_offset = 0_deg;
      m_conf->foc_encoder_inverted = false;
      m_conf->foc_encoder_ratio = 1.0;

      // Find index
      int cnt = 0;
      while(!encoder::index_found()) {
          for (radian_t i = 0.0_rad; i < 2.0 * PI_rad; i += (2.0 * PI_rad) / 500.0) {
              m_phase_now_override = i;
              chThdSleepMilliseconds(1);
          }

          cnt++;
          if (cnt > 30) {
              // Give up
              break;
          }
      }

      if (print) {
          commands::printf("Index found");
      }

      // Rotate
          for (radian_t i = 0.0_rad; i < 2.0 * PI_rad; i += (2.0 * PI_rad) / 500.0) {
          m_phase_now_override = i;
          chThdSleepMilliseconds(1);
      }

      if (print) {
          commands::printf("Rotated for sync");
      }

      // Inverted and ratio
      chThdSleepMilliseconds(1000);

      const int it_rat = 20;
      float s_sum = 0.0;
      float c_sum = 0.0;
      auto first = m_phase_now_encoder;

      for (int i = 0; i < it_rat; i++) {
          radian_t phase_old = m_phase_now_encoder;
          auto phase_ovr_tmp = m_phase_now_override;
          for (radian_t i = phase_ovr_tmp; i < phase_ovr_tmp + (2.0 / 3.0) * PI_rad;
                  i += (2.0 * PI_rad) / 500.0) {
              m_phase_now_override = i;
              chThdSleepMilliseconds(1);
          }
          norm_angle_rad(const_cast<radian_t&>(m_phase_now_override));
          chThdSleepMilliseconds(300);
          auto diff = angle_difference_rad(m_phase_now_encoder, phase_old);

          float s, c;
          sincos_rad(diff, &s, &c);
          s_sum += s;
          c_sum += c;

          if (print) {
              commands::printf("%.2f", (double)(diff * 180.0 / M_PI));
          }

          if (i > 3 && fabsf(angle_difference_rad(m_phase_now_encoder, first)) < fabsf(diff / 2.0)) {
              break;
          }
      }

      first = m_phase_now_encoder;

      for (int i = 0; i < it_rat; i++) {
          auto phase_old = m_phase_now_encoder;
          auto phase_ovr_tmp = m_phase_now_override;
          for (radian_t i = phase_ovr_tmp; i > phase_ovr_tmp - (2.0 / 3.0) * PI_rad;
                  i -= (2.0 * PI_rad) / 500.0) {
              m_phase_now_override = i;
              chThdSleepMilliseconds(1);
          }
          norm_angle_rad(const_cast<radian_t&>(m_phase_now_override));
          chThdSleepMilliseconds(300);
          auto diff = angle_difference_rad(phase_old, m_phase_now_encoder);

          float s, c;
          sincos_rad(diff, &s, &c);
          s_sum += s;
          c_sum += c;

          if (print) {
              commands::printf("%.2f", (double)(diff * 180.0 / M_PI));
          }

          if (i > 3 && fabsf(angle_difference_rad(m_phase_now_encoder, first)) < fabsf(diff / 2.0)) {
              break;
          }
      }

      float diff = atan2f(s_sum, c_sum) * 180.0 / M_PI;
      inverted = diff < 0.0;
      ratio = roundf(((2.0 / 3.0) * 180.0) / fabsf(diff));

      m_conf->foc_encoder_inverted = inverted;
      m_conf->foc_encoder_ratio = ratio;

      if (print) {
          commands::printf("Inversion and ratio detected");
      }

      // Rotate
      for (radian_t i = m_phase_now_override; i < 2.0 * PI_rad; i += (2.0 * PI_rad) / 500.0) {
          m_phase_now_override = i;
          chThdSleepMilliseconds(2);
      }

      if (print) {
          commands::printf("Rotated for sync");
          commands::printf("Enc: %.2f", (double)encoder::read_deg());
      }

      const int it_ofs = m_conf->foc_encoder_ratio * 3.0;
      s_sum = 0.0;
      c_sum = 0.0;

      for (int i = 0; i < it_ofs; i++) {
          m_phase_now_override = (i * 2.0 * PI_rad * m_conf->foc_encoder_ratio) / it_ofs;
          chThdSleepMilliseconds(500);

          auto diff = angle_difference_rad(m_phase_now_encoder, m_phase_now_override);
          float s, c;
          sincos_rad(diff, &s, &c);
          s_sum += s;
          c_sum += c;

          if (print) {
              commands::printf("%.2f", (double)(diff * 180.0 / M_PI));
          }
      }

      for (int i = it_ofs; i > 0; i--) {
          m_phase_now_override = (i * 2.0 * PI_rad * m_conf->foc_encoder_ratio) / it_ofs;
          chThdSleepMilliseconds(500);

          auto diff = angle_difference_rad(m_phase_now_encoder, m_phase_now_override);
          float s, c;
          sincos_rad(diff, &s, &c);
          s_sum += s;
          c_sum += c;

          if (print) {
              commands::printf("%.2f", (double)(diff * 180.0 / M_PI));
          }
      }
#ifdef USE_UNITS
      offset = radian_t{atan2f(s_sum, c_sum)};
#else
      offset = atan2f(s_sum, c_sum) * 180.0 / M_PI;
#endif

      if (print) {
          commands::printf("Avg: %.2f", (double)offset);
      }

      norm_angle(offset);

      if (print) {
          commands::printf("Offset detected");
      }

      // Restore configuration
      m_conf->foc_encoder_inverted = inverted_old;
      m_conf->foc_encoder_offset = offset_old;
      m_conf->foc_encoder_ratio = ratio_old;
  }

  /**
   * Lock the motor with a current and sample the voltage and current to
   * calculate the motor resistance.
   *
   * @param current
   * The locking current.
   *
   * @param samples
   * The number of samples to take.
   *
   * @return
   * The calculated motor resistance.
   */
  ohm_t measure_resistance(ampere_t current, size_t samples) {

      PhaseOverride lockedCurrentContext(0_A, current);

      // Wait for the current to rise and the motor to lock.
      chThdSleepMilliseconds(500);

      // Sample
      m_samples.reset();

      for(size_t cnt = 0;
          cnt <= 10000 && m_samples.sample_num < samples;
          cnt++)
      {
          chThdSleepMilliseconds(1);
      }

      auto const current_avg = m_samples.get_avg_current();
      auto const voltage_avg = m_samples.get_avg_voltage();

      return (voltage_avg / current_avg) * (2.0 / 3.0);
  }

  /**
   * Measure the motor inductance with short voltage pulses.
   *
   * @param duty
   * The duty cycle to use in the pulses.
   *
   * @param samples
   * The number of samples to average over.
   *
   * @param
   * The current that was used for this measurement.
   *
   * @return
   * The average d and q axis inductance in microhenry.
   */
  microhenry_t measure_inductance(float const duty, size_t const samples, ampere_t * const curr) {
      m_samples.reset();
      m_samples.measure_inductance_duty = duty;

      {
        timeout::Disabler disabledTimeout;
        mc_interface::Lock interfaceLock;

        int to_cnt = 0;
        for (size_t i = 0; i < samples; i++) {
            m_samples.measure_inductance_now = true;

            do {
                chThdSleepMicroseconds(100);
                to_cnt++;
                if (to_cnt > 50000) {
                    break;
                }
            } while (m_samples.measure_inductance_now);

            if (to_cnt > 50000) {
                break;
            }
        }

      }

      auto avg_current = m_samples.get_avg_current();
      auto avg_voltage = m_samples.get_avg_voltage();
      second_t t = (float)TIM1->ARR * m_samples.measure_inductance_duty / hw::SYSTEM_CORE_CLOCK -
                (float)(MCPWM_FOC_INDUCTANCE_SAMPLE_CNT_OFFSET + MCPWM_FOC_INDUCTANCE_SAMPLE_RISE_COMP) / hw::SYSTEM_CORE_CLOCK;

      if (curr) {
          *curr = avg_current;
      }
#ifdef USE_UNITS
      return ((avg_voltage * t) / avg_current) * (2.0 / 3.0);
#else
      return ((avg_voltage * t) / avg_current) * 1e6 * (2.0 /  3.0);
#endif
  }

  /**
   * Automatically measure the resistance and inductance of the motor with small steps.
   *
   * @param res
   * The measured resistance in ohm.
   *
   * @param ind
   * The measured inductance in microhenry.
   *
   * @return
   * True if the measurement succeeded, false otherwise.
   */
  bool measure_res_ind(ohm_t &res, microhenry_t &ind) {
      auto const f_sw_old = m_conf->foc_f_sw;
      auto const kp_old = m_conf->foc_current_kp;
      auto const ki_old = m_conf->foc_current_ki;

      m_conf->foc_f_sw = 10'000_Hz;
      m_conf->foc_current_kp = 0.01_Ohm;
      m_conf->foc_current_ki = 10_Ohm / 1_s;

      uint32_t top = hw::SYSTEM_CORE_CLOCK / m_conf->foc_f_sw;
      TIMER_UPDATE_SAMP_TOP(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);

      ohm_t res_tmp = 0_Ohm;
      ampere_t i_last = 0_A;
      for (ampere_t i = 2_A; i < (m_conf->l_current_max / 2); i *= 1.5) {
          res_tmp = measure_resistance(i, 20);

          if (i * res_tmp > 1_V) {
              i_last = i;
              break;
          }
      }

      if (i_last < 0.01_A) {
          i_last = (m_conf->l_current_max / 2.0);
      }

      res = measure_resistance(i_last, 200);

      m_conf->foc_f_sw = 3000_Hz;
      top = hw::SYSTEM_CORE_CLOCK / m_conf->foc_f_sw;
      TIMER_UPDATE_SAMP_TOP(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);

      float duty_last = 0.0;
      for (float duty = 0.02; duty < 0.5; duty *= 1.5) {
          ampere_t i_tmp;
          measure_inductance(duty, 20, &i_tmp);

          duty_last = duty;
          if (i_tmp >= i_last) {
              break;
          }
      }

      ind = measure_inductance(duty_last, 200, nullptr);

      m_conf->foc_f_sw = f_sw_old;
      m_conf->foc_current_kp = kp_old;
      m_conf->foc_current_ki = ki_old;

      top = hw::SYSTEM_CORE_CLOCK / m_conf->foc_f_sw;
      TIMER_UPDATE_SAMP_TOP(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);

      return true;
  }

  /**
   * Run the motor in open loop and figure out at which angles the hall sensors are.
   *
   * @param current
   * Current to use.
   *
   * @param hall_table
   * Table to store the result to.
   *
   * @return
   * true: Success
   * false: Something went wrong
   */
  bool hall_detect(ampere_t current, uint8_t *hall_table) {

    float sin_hall[8];
    float cos_hall[8];
    int hall_iterations[8];
    memset(sin_hall, 0, sizeof(sin_hall));
    memset(cos_hall, 0, sizeof(cos_hall));
    memset(hall_iterations, 0, sizeof(hall_iterations));

    {
        // Lock the motor
        PhaseOverride lockedCurrentContext(current, 0_A);

        chThdSleepMilliseconds(1000);

        // Forwards
        for (int i = 0; i < 3; i++) {
            for (int degrees = 0; degrees < 360; degrees++) {
#ifdef USE_UNITS
                m_phase_now_override = degree_t{(float)degrees};
#else
                m_phase_now_override = (float)degrees * M_PI / 180.0;
#endif
                chThdSleepMilliseconds(5);

                int hall = read_hall();
                float s, c;
                sincos_rad(m_phase_now_override, &s, &c);
                sin_hall[hall] += s;
                cos_hall[hall] += c;
                hall_iterations[hall]++;
            }
        }

        // Reverse
        for (int i = 0; i < 3; i++) {
            for (int degrees = 360; degrees >= 0; degrees--) {
#ifdef USE_UNITS
                m_phase_now_override = degree_t{(float)degrees};
#else
                m_phase_now_override = (float)degrees * M_PI / 180.0;
#endif
                chThdSleepMilliseconds(5);

                int hall = read_hall();
                float s, c;
                sincos_rad(m_phase_now_override, &s, &c);
                sin_hall[hall] += s;
                cos_hall[hall] += c;
                hall_iterations[hall]++;
            }
        }

      }

      int fails = 0;
      for(int i = 0;i < 8;i++) {
          if (hall_iterations[i] > 30) {
#ifdef USE_UNITS
              degree_t ang = radian_t{atan2f(sin_hall[i], cos_hall[i])};
#else
              degree_t ang = atan2f(sin_hall[i], cos_hall[i]) * 180.0 / M_PI;
#endif
              norm_angle(ang);
              hall_table[i] = (uint8_t)(ang * 200.0 / 360_deg);
          } else {
              hall_table[i] = 255;
              fails++;
          }
      }

      return fails == 2;
  }

  void print_state(void) {
      commands::printf("Mod d:        %.2f", (double)m_motor_state.mod_d);
      commands::printf("Mod q:        %.2f", (double)m_motor_state.mod_q);
      commands::printf("Duty:         %.2f", (double)m_motor_state.duty_now);
      commands::printf("Vd:           %.2f", (double)m_motor_state.vd);
      commands::printf("Vq:           %.2f", (double)m_motor_state.vq);
      commands::printf("Phase:        %.2f", (double)m_motor_state.phase);
      commands::printf("V_alpha:      %.2f", (double)m_motor_state.v_alpha);
      commands::printf("V_beta:       %.2f", (double)m_motor_state.v_beta);
      commands::printf("id:           %.2f", (double)m_motor_state.id);
      commands::printf("iq:           %.2f", (double)m_motor_state.iq);
      commands::printf("id_filter:    %.2f", (double)m_motor_state.id_filter);
      commands::printf("iq_filter:    %.2f", (double)m_motor_state.iq_filter);
      commands::printf("id_target:    %.2f", (double)m_motor_state.id_target);
      commands::printf("iq_target:    %.2f", (double)m_motor_state.iq_target);
      commands::printf("i_abs:        %.2f", (double)m_motor_state.i_abs);
      commands::printf("i_abs_filter: %.2f", (double)m_motor_state.i_abs_filter);
      commands::printf("Obs_x1:       %.2f", (double)m_observer_x1);
      commands::printf("Obs_x2:       %.2f", (double)m_observer_x2);
  }

  second_t get_last_adc_isr_duration(void) {
      return m_last_adc_isr_duration;
  }

  void tim_sample_interrupt_handler(void) {
      if (m_init_done) {
          // Generate COM event here for synchronization
          TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
      }
  }
#ifdef HW_HAS_PHASE_SHUNTS
  void read_current_measures(bool is_v7)
#else
  void read_current_measures(bool)
#endif
  {
    {
      int curr0 = ADC_Value[ADC_IND_CURR1];
      int curr1 = ADC_Value[ADC_IND_CURR2];
#ifdef HW_HAS_3_SHUNTS
      int curr2 = ADC_Value[ADC_IND_CURR3];
#else
      int curr2 = -(curr0 + curr1);
#endif

      m_curr0_sum += curr0;
      m_curr1_sum += curr1;
      m_curr2_sum += curr2;

      curr0 -= m_curr0_offset;
      curr1 -= m_curr1_offset;
      curr2 -= m_curr2_offset;

      m_curr_samples++;

      ADC_curr_norm_value[0] = curr0;
      ADC_curr_norm_value[1] = curr1;
      ADC_curr_norm_value[2] = curr2;

#ifdef HW_IS_IHM0xM1
      ADC_curr_norm_value[0] *= -1;
      ADC_curr_norm_value[1] *= -1;
      ADC_curr_norm_value[2] *= -1;
#endif
    }
      // Use the best current samples depending on the modulation state.
#ifdef HW_HAS_3_SHUNTS
    if (m_conf->foc_sample_high_current) {
          // High current sampling mode. Choose the lower currents to derive the highest one
          // in order to be able to measure higher currents.
          const float i0_abs = fabsf(ADC_curr_norm_value[0]);
          const float i1_abs = fabsf(ADC_curr_norm_value[1]);
          const float i2_abs = fabsf(ADC_curr_norm_value[2]);

          if (i0_abs > i1_abs && i0_abs > i2_abs) {
              ADC_curr_norm_value[0] = -(ADC_curr_norm_value[1] + ADC_curr_norm_value[2]);
          } else if (i1_abs > i0_abs && i1_abs > i2_abs) {
              ADC_curr_norm_value[1] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[2]);
          } else if (i2_abs > i0_abs && i2_abs > i1_abs) {
              ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
          }
      } else {
#ifdef HW_HAS_PHASE_SHUNTS
          if (m_conf->foc_sample_v0_v7 && is_v7) {
              if (TIM1->CCR1 < TIM1->CCR2 && TIM1->CCR1 < TIM1->CCR3) {
                  ADC_curr_norm_value[0] = -(ADC_curr_norm_value[1] + ADC_curr_norm_value[2]);
              } else if (TIM1->CCR2 < TIM1->CCR1 && TIM1->CCR2 < TIM1->CCR3) {
                  ADC_curr_norm_value[1] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[2]);
              } else if (TIM1->CCR3 < TIM1->CCR1 && TIM1->CCR3 < TIM1->CCR2) {
                  ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
              }
          } else {
              if (TIM1->CCR1 > TIM1->CCR2 && TIM1->CCR1 > TIM1->CCR3) {
                  ADC_curr_norm_value[0] = -(ADC_curr_norm_value[1] + ADC_curr_norm_value[2]);
              } else if (TIM1->CCR2 > TIM1->CCR1 && TIM1->CCR2 > TIM1->CCR3) {
                  ADC_curr_norm_value[1] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[2]);
              } else if (TIM1->CCR3 > TIM1->CCR1 && TIM1->CCR3 > TIM1->CCR2) {
                  ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
              }
          }
#else
          if (TIM1->CCR1 > TIM1->CCR2 && TIM1->CCR1 > TIM1->CCR3) {
              ADC_curr_norm_value[0] = -(ADC_curr_norm_value[1] + ADC_curr_norm_value[2]);
          } else if (TIM1->CCR2 > TIM1->CCR1 && TIM1->CCR2 > TIM1->CCR3) {
              ADC_curr_norm_value[1] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[2]);
          } else if (TIM1->CCR3 > TIM1->CCR1 && TIM1->CCR3 > TIM1->CCR2) {
              ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
          }
#endif
      }
#endif

  }

  void inductance_measure_sequence()
  {
    static int inductance_state = 0;
    const uint32_t duty_cnt = (uint32_t)((float)TIM1->ARR * m_samples.measure_inductance_duty);
    const uint32_t samp_time = duty_cnt - MCPWM_FOC_INDUCTANCE_SAMPLE_CNT_OFFSET;

    switch(inductance_state) {
    case 0:
        TIMER_UPDATE_DUTY_SAMP(0, 0, 0, samp_time);
        start_pwm_hw();
        break;
    case 2:
        TIMER_UPDATE_DUTY(duty_cnt, 0, duty_cnt);
        break;
    case 3:
        m_samples.avg_current_tot -= ADC_curr_norm_value[1] * FAC_CURRENT;
        m_samples.avg_voltage_tot += GET_INPUT_VOLTAGE();
        m_samples.sample_num++;
        TIMER_UPDATE_DUTY(0, 0, 0);
        break;
    case 4:
        TIMER_UPDATE_DUTY(0, duty_cnt, duty_cnt);
        break;
    case 6:
        m_samples.avg_current_tot -= ADC_curr_norm_value[0] * FAC_CURRENT;
        m_samples.avg_voltage_tot += GET_INPUT_VOLTAGE();
        m_samples.sample_num++;
        TIMER_UPDATE_DUTY(0, 0, 0);
        break;
    case 8:
#ifdef HW_HAS_3_SHUNTS
        TIMER_UPDATE_DUTY(duty_cnt, duty_cnt, 0);
#else
        TIMER_UPDATE_DUTY(0, 0, duty_cnt);
#endif
        break;
    case 9:
        m_samples.avg_current_tot -= ADC_curr_norm_value[2] * FAC_CURRENT;
        m_samples.avg_voltage_tot += GET_INPUT_VOLTAGE();
        m_samples.sample_num++;
        stop_pwm_hw();
        TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);
        break;
    case 10:
        inductance_state = 0;
        m_samples.measure_inductance_now = false;
        return;
    default:
        break;
    }

    inductance_state++;
    return;
  }

  void adc_interrupt_handler(void *p, uint32_t flags) {
      (void)p;
      (void)flags;

      TIM12->CNT = 0;

      bool const is_v7 = !(TIM1->CR1 & TIM_CR1_DIR);

      if (!m_samples.measure_inductance_now) {
#ifdef HW_HAS_PHASE_SHUNTS
          if (!m_conf->foc_sample_v0_v7 && is_v7) {
              return;
          }
#else
          if (is_v7) {
              return;
          }
#endif
      }

      // Reset the watchdog
      WWDG_SetCounter(100);

      read_current_measures(is_v7);

      if (m_samples.measure_inductance_now) {
        if (is_v7) {
            inductance_measure_sequence();
        }
        return;
      }

  #ifdef HW_HAS_PHASE_SHUNTS
      second_t dt;
      if (m_conf->foc_sample_v0_v7) {
          dt = 1.0 / m_conf->foc_f_sw;
      } else {
          dt = 2.0 / m_conf->foc_f_sw;
      }
  #else
      second_t const dt = 2.0 / m_conf->foc_f_sw;
  #endif

      UTILS_LP_FAST(m_motor_state.v_bus, GET_INPUT_VOLTAGE(), 0.1);

      degree_t enc_ang = 0_deg;
      if (encoder::is_configured()) {
          enc_ang = encoder::read_deg();
          degree_t phase_tmp = enc_ang;
          if (m_conf->foc_encoder_inverted) {
              phase_tmp = 360_deg - phase_tmp;
          }
          phase_tmp *= m_conf->foc_encoder_ratio;
          phase_tmp -= m_conf->foc_encoder_offset;
          norm_angle(const_cast<degree_t&>(phase_tmp));
#ifdef USE_UNITS
          m_phase_now_encoder = phase_tmp;
#else
          m_phase_now_encoder = phase_tmp * (M_PI / 180.0);
#endif
      }

      if (m_state == MC_STATE_RUNNING) {
          run_foc_control(dt);

      } else { // m_state != MC_STATE_RUNNING
          // Track back emf
          track_bemf(dt);
      }

      // Calculate duty cycle
      m_motor_state.duty_now = SIGN(m_motor_state.vq) *
              sqrtf(m_motor_state.mod_d * m_motor_state.mod_d +
                    m_motor_state.mod_q * m_motor_state.mod_q) / SQRT3_BY_2;

      // Run PLL for speed estimation
      pll_run(m_motor_state.phase, dt, m_pll_phase, m_pll_speed);

      // Update tachometer (resolution = 60 deg as for BLDC)
      auto ph_tmp = m_motor_state.phase;
      norm_angle_rad(ph_tmp);
      int step = (int)floorf((ph_tmp + PI_rad) / (2.0 * PI_rad) * 6.0);
      truncate_number(step, 0, 5);
      static int step_last = 0;
      int diff = step - step_last;
      step_last = step;

      if (diff > 3) {
          diff -= 6;
      } else if (diff < -2) {
          diff += 6;
      }

      m_tachometer += diff;
      m_tachometer_abs += abs(diff);

      // Track position control angle
      // TODO: Have another look at this.
      degree_t angle_now = 0_deg;
      if (encoder::is_configured()) {
          angle_now = enc_ang;
      } else {
#ifdef USE_UNITS
          angle_now = m_motor_state.phase;
#else
          angle_now = m_motor_state.phase * (180.0 / M_PI);
#endif
      }

      if (m_conf->p_pid_ang_div > 0.98 && m_conf->p_pid_ang_div < 1.02) {
          m_pos_pid_now = angle_now;
      } else {
          static degree_t angle_last = 0_deg;
          auto diff_f = angle_difference(angle_now, angle_last);
          angle_last = angle_now;
          m_pos_pid_now += diff_f / m_conf->p_pid_ang_div;
          norm_angle(const_cast<degree_t&>(m_pos_pid_now));
      }

      // Run position control
      if (m_state == MC_STATE_RUNNING) {
          run_pid_control_pos(m_pos_pid_now, m_pos_pid_set, dt);
      }

      // MCIF handler
      mc_interface::collect_mc_state_samples();

      m_last_adc_isr_duration = TIM12->CNT / TIM12_FREQ;
  }

  void run_foc_control(second_t dt){

    static radian_t phase_before = 0.0_rad;
    auto const phase_diff = angle_difference_rad(m_motor_state.phase, phase_before);
    phase_before = m_motor_state.phase;

    ampere_t ia = ADC_curr_norm_value[0] * FAC_CURRENT;
    ampere_t ib = ADC_curr_norm_value[1] * FAC_CURRENT;
//    float ic = -(ia + ib);

    // Clarke transform assuming balanced currents
    m_motor_state.i_alpha = ia;
    m_motor_state.i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;

    // Full Clarke transform in case there are current offsets
//        m_motor_state.i_alpha = (2.0 / 3.0) * ia - (1.0 / 3.0) * ib - (1.0 / 3.0) * ic;
//        m_motor_state.i_beta = ONE_BY_SQRT3 * ib - ONE_BY_SQRT3 * ic;

    const float duty_abs = fabsf(m_motor_state.duty_now);
    auto id_set_tmp = m_id_set;
    auto iq_set_tmp = m_iq_set;
    m_motor_state.max_duty = m_conf->l_max_duty;

    static float duty_filtered = 0.0;
    UTILS_LP_FAST(duty_filtered, m_motor_state.duty_now, 0.1);
    truncate_number_abs(duty_filtered, 1.0);

    float duty_set = m_duty_cycle_set;
    bool control_duty = m_control_mode == CONTROL_MODE_DUTY;

    // When the filtered duty cycle in sensorless mode becomes low in brake mode, the
    // observer has lost tracking. Use duty cycle control with the lowest duty cycle
    // to get as smooth braking as possible.
    if (m_control_mode == CONTROL_MODE_CURRENT_BRAKE
            //                && (m_conf->foc_sensor_mode != FOC_SENSOR_MODE_ENCODER) // Don't use this with encoders
            && fabsf(duty_filtered) < 0.03) {
        control_duty = true;
        duty_set = 0.0;
    }

    // Brake when set ERPM is below min ERPM
    if (m_control_mode == CONTROL_MODE_SPEED &&
            fabsf(m_speed_pid_set_rpm) < m_conf->s_pid_min_erpm) {
        control_duty = true;
        duty_set = 0.0;
    }

    if (control_duty) {
        // Duty cycle control
        static scalar_t duty_i_term = 0.0;

        if (fabsf(duty_set) < (duty_abs - 0.05) ||
            SIGN(m_motor_state.vq) * m_motor_state.iq < m_conf->lo_current_min) {
            // Truncating the duty cycle here would be dangerous, so run a PID controller.

            // Compensation for supply voltage variations
            float scale = 1_V / GET_INPUT_VOLTAGE();

            // Compute error
            float error = duty_set - m_motor_state.duty_now;

            // Compute parameters
            float p_term = error * m_conf->foc_duty_dowmramp_kp * scale;
            duty_i_term += error * m_conf->foc_duty_dowmramp_ki * dt * scale;

            // I-term wind-up protection
            truncate_number_abs(duty_i_term, 1.0);

            // Calculate output
            float output = p_term + duty_i_term;
            truncate_number_abs(output, 1.0);
            iq_set_tmp = output * m_conf->lo_current_max;
        } else {
            // If the duty cycle is less than or equal to the set duty cycle just limit
            // the modulation and use the maximum allowed current.
            duty_i_term = 0.0;
            m_motor_state.max_duty = duty_set;

            iq_set_tmp = SIGN(duty_set) * m_conf->lo_current_max;
        }
    } else if (m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
        // Braking
        iq_set_tmp = fabsf(iq_set_tmp);

        if (phase_diff > 0.0_rad) {
            iq_set_tmp = -iq_set_tmp;
        } else if (phase_diff == 0.0_rad) {
            iq_set_tmp = 0_A;
        }
    }

    // Run observer
    if (!m_phase_override) {
        observer_update(m_motor_state.v_alpha, m_motor_state.v_beta,
                m_motor_state.i_alpha, m_motor_state.i_beta, dt,
                m_observer_x1, m_observer_x2, m_phase_now_observer);
    }

    switch (m_conf->foc_sensor_mode) {
    case FOC_SENSOR_MODE_ENCODER:
        if (encoder::index_found()) {
            m_motor_state.phase = correct_encoder(m_phase_now_observer, m_phase_now_encoder, m_pll_speed);
        } else {
            // Rotate the motor in open loop if the index isn't found.
            m_motor_state.phase = m_phase_now_encoder_no_index;
        }

        if (!m_phase_override) {
            id_set_tmp = 0_A;
        }
        break;
    case FOC_SENSOR_MODE_HALL:
        m_phase_now_observer = correct_hall(m_phase_now_observer, m_pll_speed, dt);
        m_motor_state.phase = m_phase_now_observer;

        if (!m_phase_override) {
            id_set_tmp = 0_A;
        }
        break;
    case FOC_SENSOR_MODE_SENSORLESS:
        if (m_phase_observer_override) {
            m_motor_state.phase = m_phase_now_observer_override;
        } else {
            m_motor_state.phase = m_phase_now_observer;
        }

        // Inject D axis current at low speed to make the observer track
        // better. This does not seem to be necessary with dead time
        // compensation.
        // Note: this is done at high rate to prevent noise.
        if (!m_phase_override) {
            if (duty_abs < m_conf->foc_sl_d_current_duty) {
                id_set_tmp = map(duty_abs,
                                 0.0,
                                 m_conf->foc_sl_d_current_duty,
                                 fabsf(m_motor_state.iq_target) * m_conf->foc_sl_d_current_factor,
                                 0_A);
            } else {
                id_set_tmp = 0_A;
            }
        }
        break;
    }

    // Force the phase to 0 in handbrake mode so that the current simply locks the rotor.
    if (m_control_mode == CONTROL_MODE_HANDBRAKE) {
        m_motor_state.phase = 0.0_rad;
    } else if (m_control_mode == CONTROL_MODE_OPENLOOP) {
        static radian_t openloop_angle = 0.0_rad;
        openloop_angle += dt * m_openloop_speed;
        norm_angle_rad(openloop_angle);
        m_motor_state.phase = openloop_angle;
    }

    if (m_phase_override) {
        m_motor_state.phase = m_phase_now_override;
    }

    // Apply current limits
    // TODO: Consider D axis current for the input current as well.
    auto const mod_q = m_motor_state.mod_q;
    if (mod_q > 0.001) {
        truncate_number(iq_set_tmp, m_conf->lo_in_current_min / mod_q,
                                    m_conf->lo_in_current_max / mod_q);
    } else if (mod_q < -0.001) {
        truncate_number(iq_set_tmp, m_conf->lo_in_current_max / mod_q,
                                    m_conf->lo_in_current_min / mod_q);
    }

    if (mod_q > 0.0) {
        truncate_number(iq_set_tmp, m_conf->lo_current_min,
                                    m_conf->lo_current_max);
    } else {
        truncate_number(iq_set_tmp, -m_conf->lo_current_max,
                                    -m_conf->lo_current_min);
    }

    saturate_vector_2d(id_set_tmp, iq_set_tmp,
            max_abs(m_conf->lo_current_max, m_conf->lo_current_min));

    m_motor_state.id_target = id_set_tmp;
    m_motor_state.iq_target = iq_set_tmp;

    control_current(m_motor_state, dt);
  }

  void track_bemf(second_t dt){
#ifdef HW_HAS_3_SHUNTS
      auto Va = GET_BEMF_VOLTAGE_CH(ADC_IND_SENS1);
      auto Vb = GET_BEMF_VOLTAGE_CH(ADC_IND_SENS2);
      auto Vc = GET_BEMF_VOLTAGE_CH(ADC_IND_SENS3);
#else
      auto Va = GET_BEMF_VOLTAGE_CH(ADC_IND_SENS1);
      auto Vb = GET_BEMF_VOLTAGE_CH(ADC_IND_SENS3);
      auto Vc = GET_BEMF_VOLTAGE_CH(ADC_IND_SENS2);
#endif

      // Full Clarke transform (no balanced voltages)
      m_motor_state.v_alpha = (2.0 / 3.0) * Va - (1.0 / 3.0) * Vb - (1.0 / 3.0) * Vc;
      m_motor_state.v_beta = ONE_BY_SQRT3 * Vb - ONE_BY_SQRT3 * Vc;

      float c, s;
      fast_sincos_better(m_motor_state.phase, &s, &c);

      // Park transform
      auto vd_tmp = c * m_motor_state.v_alpha + s * m_motor_state.v_beta;
      auto vq_tmp = c * m_motor_state.v_beta  - s * m_motor_state.v_alpha;

      UTILS_NAN_ZERO(m_motor_state.vd);
      UTILS_NAN_ZERO(m_motor_state.vq);

      UTILS_LP_FAST(m_motor_state.vd, vd_tmp, 0.2);
      UTILS_LP_FAST(m_motor_state.vq, vq_tmp, 0.2);

      m_motor_state.vd_int = m_motor_state.vd;
      m_motor_state.vq_int = m_motor_state.vq;

      // Update corresponding modulation
      m_motor_state.mod_d = m_motor_state.vd / ((2.0 / 3.0) * m_motor_state.v_bus);
      m_motor_state.mod_q = m_motor_state.vq / ((2.0 / 3.0) * m_motor_state.v_bus);

      // The current is 0 when the motor is undriven
      m_motor_state.i_alpha = 0_A;
      m_motor_state.i_beta = 0_A;
      m_motor_state.id = 0_A;
      m_motor_state.iq = 0_A;
      m_motor_state.id_filter = 0_A;
      m_motor_state.iq_filter = 0_A;
      m_motor_state.i_bus = 0_A;
      m_motor_state.i_abs = 0_A;
      m_motor_state.i_abs_filter = 0_A;

      // Run observer
      observer_update(m_motor_state.v_alpha, m_motor_state.v_beta,
              m_motor_state.i_alpha, m_motor_state.i_beta, dt,
              m_observer_x1,
              m_observer_x2,
              m_phase_now_observer);

      switch (m_conf->foc_sensor_mode) {
      case FOC_SENSOR_MODE_ENCODER:
          m_motor_state.phase = correct_encoder(m_phase_now_observer, m_phase_now_encoder, m_pll_speed);
          break;
      case FOC_SENSOR_MODE_HALL:
          m_phase_now_observer = correct_hall(m_phase_now_observer, m_pll_speed, dt);
          m_motor_state.phase = m_phase_now_observer;
          break;
      case FOC_SENSOR_MODE_SENSORLESS:
          m_motor_state.phase = m_phase_now_observer;
          break;
      }
  }

  THD_FUNCTION(timer_thread, arg) {
      (void)arg;

      chRegSetThreadName("mcpwm_foc timer");

      for(;;) {
          if (m_timer_thd_stop) {
              m_timer_thd_stop = false;
              return;
          }

          auto openloop_rpm = map(fabsf(m_motor_state.iq_target),
                                        0_A,
                                        m_conf->l_current_max,
                                        0_rpm,
                                        m_conf->foc_openloop_rpm);

          truncate_number_abs(openloop_rpm, m_conf->foc_openloop_rpm);

          const second_t dt = 0.001_s;
#ifdef USE_UNITS
          radians_per_second_t const min_rads = openloop_rpm;
#else
          radians_per_second_t const min_rads = (openloop_rpm * 2.0 * M_PI) / 60.0;
#endif
          static second_t min_rpm_hyst_timer = 0.0_s;
          static second_t min_rpm_timer = 0.0_s;

          radian_t add_min_speed = 0.0_rad;
          if (m_motor_state.duty_now > 0.0) {
              add_min_speed = min_rads * dt;
          } else {
              add_min_speed = -min_rads * dt;
          }

          // Open loop encoder angle for when the index is not found
          m_phase_now_encoder_no_index += add_min_speed;
          norm_angle_rad(const_cast<radian_t&>(m_phase_now_encoder_no_index));

          // Output a minimum speed from the observer
          if (fabsf(m_pll_speed) < min_rads) {
              min_rpm_hyst_timer += dt;
          } else if (min_rpm_hyst_timer > 0.0_s) {
              min_rpm_hyst_timer -= dt;
          }

          // Don't use this in brake mode.
          if (m_control_mode == CONTROL_MODE_CURRENT_BRAKE || fabsf(m_motor_state.duty_now) < 0.001) {
              min_rpm_hyst_timer = 0.0_s;
              m_phase_observer_override = false;
          }

          bool started_now = false;
          if (min_rpm_hyst_timer > m_conf->foc_sl_openloop_hyst && min_rpm_timer <= 0.0001_s) {
              min_rpm_timer = m_conf->foc_sl_openloop_time;
              started_now = true;
          }

          if (min_rpm_timer > 0.0_s) {
              m_phase_now_observer_override += add_min_speed;

              // When the motor gets stuck it tends to be 90 degrees off, so start the open loop
              // sequence by correcting with 90 degrees.
              if (started_now) {
                  if (m_motor_state.duty_now > 0.0) {
                      m_phase_now_observer_override += PI_rad / 2.0;
                  } else {
                      m_phase_now_observer_override -= PI_rad / 2.0;
                  }
              }

              norm_angle_rad(const_cast<radian_t&>(m_phase_now_observer_override));
              m_phase_observer_override = true;
              min_rpm_timer -= dt;
              min_rpm_hyst_timer = 0.0_s;
          } else {
              m_phase_now_observer_override = m_phase_now_observer;
              m_phase_observer_override = false;
          }

          // Samples
          if (m_state == MC_STATE_RUNNING) {
              auto const volatil_ vd_tmp = m_motor_state.vd;
              auto const volatil_ vq_tmp = m_motor_state.vq;
              auto const volatil_ id_tmp = m_motor_state.id;
              auto const volatil_ iq_tmp = m_motor_state.iq;

              m_samples.avg_current_tot += units::math::sqrt(SQ(id_tmp) + SQ(iq_tmp));
              m_samples.avg_voltage_tot += units::math::sqrt(SQ(vd_tmp) + SQ(vq_tmp));
              m_samples.sample_num++;
          }

          // Update and the observer gain.
          m_gamma_now = map(fabsf(m_motor_state.duty_now), 0.0, 1.0,
                            m_conf->foc_observer_gain * m_conf->foc_observer_gain_slow,
                            m_conf->foc_observer_gain);

          run_pid_control_speed(dt);
          chThdSleepMilliseconds(1);
      }

  }

  void do_dc_cal(void) {
      DCCAL_ON();

      // Wait max 5 seconds
      int cnt = 0;
      while(IS_DRV_FAULT()){
          chThdSleepMilliseconds(1);
          cnt++;
          if (cnt > 5000) {
              break;
          }
      };

      chThdSleepMilliseconds(1000);
      m_curr0_sum = 0;
      m_curr1_sum = 0;
      m_curr2_sum = 0;

      m_curr_samples = 0;
      while(m_curr_samples < 4000) {};
      m_curr0_offset = m_curr0_sum / m_curr_samples;
      m_curr1_offset = m_curr1_sum / m_curr_samples;
      m_curr2_offset = m_curr2_sum / m_curr_samples;

      DCCAL_OFF();
      m_dccal_done = true;
  }

  // See http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
  void observer_update(volt_t const v_alpha,
                       volt_t const v_beta,
                       ampere_t const i_alpha,
                       ampere_t const i_beta,
                       second_t const dt,
                       volatil_ weber_t& x1,
                       volatil_ weber_t& x2,
                       volatil_ radian_t& phase) {

      auto const L = (3.0 / 2.0) * m_conf->foc_motor_l;
      auto const lambda = m_conf->foc_motor_flux_linkage;
      auto R       = (3.0 / 2.0) * m_conf->foc_motor_r;

      {
        // Saturation compensation
        const float sign = SIGN(m_motor_state.iq * m_motor_state.vq);
        R -= R * sign * m_conf->foc_sat_comp * (m_motor_state.i_abs_filter / m_conf->l_current_max);

        // Temperature compensation
        celsius_t const temp = mc_interface::temp_motor_filtered();
        if (m_conf->foc_temp_comp && temp > -1 * 5_degC) {
            R += R * 0.00386 / 1_degC * (temp - m_conf->foc_temp_comp_base_temp);
        }
      }

      auto const L_ia = L * i_alpha;
      auto const L_ib = L * i_beta;
      auto const R_ia = R * i_alpha;
      auto const R_ib = R * i_beta;
      auto const lambda_2 = SQ(lambda);
      auto const gamma_half = m_gamma_now * 0.5;

      // Original
  //	float err = lambda_2 - (SQ(*x1 - L_ia) + SQ(*x2 - L_ib));
  //	float x1_dot = -R_ia + v_alpha + gamma_half * (*x1 - L_ia) * err;
  //	float x2_dot = -R_ib + v_beta + gamma_half * (*x2 - L_ib) * err;
  //	*x1 += x1_dot * dt;
  //	*x2 += x2_dot * dt;

      // Iterative with some trial and error
      const int iterations = 6;
      second_t const dt_iteration = dt / iterations;
      for (int i = 0;i < iterations;i++) {
          auto err = lambda_2 - (SQ(x1 - L_ia) + SQ(x2 - L_ib));
          auto gamma_tmp = gamma_half;
          if (truncate_number_abs(err, lambda_2 * 0.2)) {
              gamma_tmp *= 10.0;
          }
          auto x1_dot = -R_ia + v_alpha + gamma_tmp * (x1 - L_ia) * err;
          auto x2_dot = -R_ib + v_beta  + gamma_tmp * (x2 - L_ib) * err;

          x1 += x1_dot * dt_iteration;
          x2 += x2_dot * dt_iteration;
      }

      // Same as above, but without iterations.
  //	float err = lambda_2 - (SQ(x1 - L_ia) + SQ(x2 - L_ib));
  //	float gamma_tmp = gamma_half;
  //	if (truncate_number_abs(err, lambda_2 * 0.2)) {
  //		gamma_tmp *= 10.0;
  //	}
  //	float x1_dot = -R_ia + v_alpha + gamma_tmp * (x1 - L_ia) * err;
  //	float x2_dot = -R_ib + v_beta + gamma_tmp * (x2 - L_ib) * err;
  //	x1 += x1_dot * dt;
  //	x2 += x2_dot * dt;

      UTILS_NAN_ZERO(x1);
      UTILS_NAN_ZERO(x2);

      phase = fast_atan2(x2 - L_ib, x1 - L_ia);
  }

  void pll_run(radian_t const phase,
               second_t const dt,
               volatil_ radian_t& phase_var,
               volatil_ radians_per_second_t& speed_var) {
      UTILS_NAN_ZERO(phase_var);
      auto delta_theta = phase - phase_var;
      norm_angle_rad(delta_theta);
      UTILS_NAN_ZERO(speed_var);
      phase_var += (speed_var + m_conf->foc_pll_kp * delta_theta) * dt;
      norm_angle_rad(const_cast<radian_t&>(phase_var));
      speed_var += m_conf->foc_pll_ki * delta_theta * dt;
  }

  /**
   * Run the current control loop.
   *
   * @param state_m
   * The motor state.
   *
   * Parameters that shall be set before calling this function:
   * id_target
   * iq_target
   * max_duty
   * phase
   * i_alpha
   * i_beta
   * v_bus
   *
   * Parameters that will be updated in this function:
   * i_bus
   * i_abs
   * i_abs_filter
   * v_alpha
   * v_beta
   * mod_d
   * mod_q
   * id
   * iq
   * id_filter
   * iq_filter
   * vd
   * vq
   * vd_int
   * vq_int
   * svm_sector
   *
   * @param dt
   * The time step in seconds.
   */
  void control_current(volatil_ motor_state_t& state_m, second_t dt) {
      float c,s;
      fast_sincos_better(const_cast<motor_state_t&>(state_m).phase, &s, &c);

      float max_duty = fabsf(state_m.max_duty);
      truncate_number(max_duty, 0.0, m_conf->l_max_duty);

      state_m.id = c * state_m.i_alpha + s * state_m.i_beta;
      state_m.iq = c * state_m.i_beta  - s * state_m.i_alpha;
      UTILS_LP_FAST(state_m.id_filter, state_m.id, m_conf->foc_current_filter_const);
      UTILS_LP_FAST(state_m.iq_filter, state_m.iq, m_conf->foc_current_filter_const);

      ampere_t Ierr_d = state_m.id_target - state_m.id;
      ampere_t Ierr_q = state_m.iq_target - state_m.iq;

      state_m.vd = state_m.vd_int + Ierr_d * m_conf->foc_current_kp;
      state_m.vq = state_m.vq_int + Ierr_q * m_conf->foc_current_kp;

      // Temperature compensation
      {
        auto const temp_motor = mc_interface::temp_motor_filtered();
        auto ki = m_conf->foc_current_ki;
        if (m_conf->foc_temp_comp && temp_motor > -1 * 5_degC) {
            ki += ki * (0.00386/1_degC) * (temp_motor - m_conf->foc_temp_comp_base_temp);
        }

        state_m.vd_int += Ierr_d * ki * dt;
        state_m.vq_int += Ierr_q * ki * dt;
      }

      // Saturation
      saturate_vector_2d(const_cast<volt_t&>(state_m.vd),
                         const_cast<volt_t&>(state_m.vq),
                         (2.0 / 3.0) * max_duty * SQRT3_BY_2 * state_m.v_bus);

      state_m.mod_d = state_m.vd / ((2.0 / 3.0) * state_m.v_bus);
      state_m.mod_q = state_m.vq / ((2.0 / 3.0) * state_m.v_bus);

      // Windup protection
  //	saturate_vector_2d((float&)state_m.vd_int, (float&)state_m.vq_int,
  //			(2.0 / 3.0) * max_duty * SQRT3_BY_2 * state_m.v_bus);
      truncate_number_abs(const_cast<volt_t&>(state_m.vd_int), (2.0 / 3.0) * max_duty * SQRT3_BY_2 * state_m.v_bus);
      truncate_number_abs(const_cast<volt_t&>(state_m.vq_int), (2.0 / 3.0) * max_duty * SQRT3_BY_2 * state_m.v_bus);

      // TODO: Have a look at this?
      state_m.i_bus = state_m.mod_d * state_m.id + state_m.mod_q * state_m.iq;
      state_m.i_abs = units::math::sqrt(SQ(state_m.id) + SQ(state_m.iq));
      state_m.i_abs_filter = units::math::sqrt(SQ(state_m.id_filter) + SQ(state_m.iq_filter));

      float mod_alpha = c * state_m.mod_d - s * state_m.mod_q;
      float mod_beta  = c * state_m.mod_q + s * state_m.mod_d;

      // Deadtime compensation
      ampere_t const i_alpha_filter = c * state_m.id_target - s * state_m.iq_target;
      ampere_t const i_beta_filter  = c * state_m.iq_target + s * state_m.id_target;
      ampere_t const ia_filter = i_alpha_filter;
      ampere_t const ib_filter = -0.5 * i_alpha_filter + SQRT3_BY_2 * i_beta_filter;
      ampere_t const ic_filter = -0.5 * i_alpha_filter - SQRT3_BY_2 * i_beta_filter;
      float const mod_alpha_filter_sgn = (2.0 / 3.0) * SIGN(ia_filter) - (1.0 / 3.0) * SIGN(ib_filter) - (1.0 / 3.0) * SIGN(ic_filter);
      float const mod_beta_filter_sgn = ONE_BY_SQRT3 * SIGN(ib_filter) - ONE_BY_SQRT3 * SIGN(ic_filter);
#ifdef USE_UNITS
      float const mod_comp_fact = m_conf->foc_dt_us * m_conf->foc_f_sw;
#else
      float const mod_comp_fact = m_conf->foc_dt_us * m_conf->foc_f_sw * 1e-6;
#endif
      float const mod_alpha_comp = mod_alpha_filter_sgn * mod_comp_fact;
      float const mod_beta_comp  = mod_beta_filter_sgn  * mod_comp_fact;

      // Apply compensation here so that 0 duty cycle has no glitches.
      state_m.v_alpha = (mod_alpha - mod_alpha_comp) * (2.0 / 3.0) * state_m.v_bus;
      state_m.v_beta  = (mod_beta  - mod_beta_comp)  * (2.0 / 3.0) * state_m.v_bus;

      // Set output (HW Dependent)
      uint32_t duty1, duty2, duty3, top;
      top = TIM1->ARR;
      svm(-mod_alpha, -mod_beta, top, duty1, duty2, duty3, (uint32_t&)state_m.svm_sector);
      TIMER_UPDATE_DUTY(duty1, duty2, duty3);

      if (!m_output_on) {
          start_pwm_hw();
      }
  }

  // Magnitude must not be larger than sqrt(3)/2, or 0.866
  void svm(float const alpha, float const beta, uint32_t const PWMHalfPeriod,
          uint32_t& tAout, uint32_t& tBout, uint32_t& tCout, uint32_t& svm_sector) {
      uint32_t sector;

      if (beta >= 0.0f) {
          if (alpha >= 0.0f) {
              //quadrant I
              if (ONE_BY_SQRT3 * beta > alpha) {
                  sector = 2;
              } else {
                  sector = 1;
              }
          } else {
              //quadrant II
              if (-ONE_BY_SQRT3 * beta > alpha) {
                  sector = 3;
              } else {
                  sector = 2;
              }
          }
      } else {
          if (alpha >= 0.0f) {
              //quadrant IV5
              if (-ONE_BY_SQRT3 * beta > alpha) {
                  sector = 5;
              } else {
                  sector = 6;
              }
          } else {
              //quadrant III
              if (ONE_BY_SQRT3 * beta > alpha) {
                  sector = 4;
              } else {
                  sector = 5;
              }
          }
      }

      // PWM timings
      uint32_t tA, tB, tC;

      switch (sector) {

      // sector 1-2
      case 1: {
          // Vector on-times
          uint32_t t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
          uint32_t t2 = (        TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

          // PWM timings
          tA = (PWMHalfPeriod - t1 - t2) / 2;
          tB = tA + t1;
          tC = tB + t2;

          break;
      }

      // sector 2-3
      case 2: {
          // Vector on-times
          uint32_t t2 = ( alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
          uint32_t t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

          // PWM timings
          tB = (PWMHalfPeriod - t2 - t3) / 2;
          tA = tB + t3;
          tC = tA + t2;

          break;
      }

      // sector 3-4
      case 3: {
          // Vector on-times
          uint32_t t3 = (         TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
          uint32_t t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

          // PWM timings
          tB = (PWMHalfPeriod - t3 - t4) / 2;
          tC = tB + t3;
          tA = tC + t4;

          break;
      }

      // sector 4-5
      case 4: {
          // Vector on-times
          uint32_t t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
          uint32_t t5 = (        -TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

          // PWM timings
          tC = (PWMHalfPeriod - t4 - t5) / 2;
          tB = tC + t5;
          tA = tB + t4;

          break;
      }

      // sector 5-6
      case 5: {
          // Vector on-times
          uint32_t t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
          uint32_t t6 = ( alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

          // PWM timings
          tC = (PWMHalfPeriod - t5 - t6) / 2;
          tA = tC + t5;
          tB = tA + t6;

          break;
      }

      // sector 6-1
      case 6: {
          // Vector on-times
          uint32_t t6 = (       -TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
          uint32_t t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

          // PWM timings
          tA = (PWMHalfPeriod - t6 - t1) / 2;
          tC = tA + t1;
          tB = tC + t6;

          break;
      }
      }

      tAout = tA;
      tBout = tB;
      tCout = tC;
      svm_sector = sector;
  }

  void run_pid_control_pos(degree_t angle_now, degree_t angle_set, second_t dt) {
      static degree_t i_term = 0_deg;
      static degree_t prev_error = 0_deg;
      degree_t p_term;
      degree_t d_term;

      // PID is off. Return.
      if (m_control_mode != CONTROL_MODE_POS) {
          i_term = 0_deg;
          prev_error = 0_deg;
          return;
      }

      // Compute parameters
      auto error = angle_difference(angle_set, angle_now);

      if (encoder::is_configured()) {
          if (m_conf->foc_encoder_inverted) {
              error = -error;
          }
      }

      p_term  = error *  m_conf->p_pid_kp;
      i_term += error * (m_conf->p_pid_ki * dt);

      {
        // Average DT for the D term when the error does not change. This likely
        // happens at low speed when the position resolution is low and several
        // control iterations run without position updates.
        // TODO: Are there problems with this approach?
        static second_t dt_int = 0.0_s;
        dt_int += dt;
        if (error == prev_error) {
            d_term = 0_deg;
        } else {
            d_term = (error - prev_error) * (m_conf->p_pid_kd / dt_int);
            dt_int = 0.0_s;
        }
      }

      // Filter D
      static degree_t d_filter = 0_deg;
      UTILS_LP_FAST(d_filter, d_term, m_conf->p_pid_kd_filter);
      d_term = d_filter;


      // I-term wind-up protection
      truncate_number_abs(p_term, 1_deg);
      truncate_number_abs(i_term, 1_deg - fabsf(p_term));

      // Store previous error
      prev_error = error;

      // Calculate output
      auto output = static_cast<float>(p_term + i_term + d_term);
      truncate_number_abs(output, 1.0);

      if (encoder::is_configured()) {
          if (encoder::index_found()) {
              m_iq_set = output * m_conf->lo_current_max;
          } else {
              // Rotate the motor with 40 % power until the encoder index is found.
              m_iq_set =    0.4 * m_conf->lo_current_max;
          }
      } else {
          m_iq_set = output * m_conf->lo_current_max;
      }
  }

  void run_pid_control_speed(second_t dt) {
      static rpm_t i_term = 0_rpm;
      static rpm_t prev_error = 0_rpm;
      rpm_t p_term;
      rpm_t d_term;

      // PID is off. Return.
      if (m_control_mode != CONTROL_MODE_SPEED) {
          i_term = 0_rpm;
          prev_error = 0_rpm;
          return;
      }

      auto const rpm = get_rpm();
      auto error = m_speed_pid_set_rpm - rpm;

      // Too low RPM set. Reset state and return.
      if (fabsf(m_speed_pid_set_rpm) < m_conf->s_pid_min_erpm) {
          i_term = 0_rpm;
          prev_error = error;
          return;
      }

      // Compute parameters
      p_term  = error               * m_conf->s_pid_kp      / 20.0;
      i_term += error               * m_conf->s_pid_ki * dt / 20.0;
      d_term  =(error - prev_error) * m_conf->s_pid_kd / dt / 20.0;

      // Filter D
      static rpm_t d_filter = 0_rpm;
      UTILS_LP_FAST(d_filter, d_term, m_conf->s_pid_kd_filter);
      d_term = d_filter;

      // I-term wind-up protection
      truncate_number_abs(i_term, 1_rpm);

      // Store previous error
      prev_error = error;

      // Calculate output
      auto output = static_cast<float>(p_term + i_term + d_term);
      truncate_number_abs(output, 1);

      // Optionally disable braking
      if (!m_conf->s_pid_allow_braking) {
          if (rpm > 0_rpm && output < 0.0) {
              output = 0.0;
          }

          if (rpm < 0_rpm && output > 0.0) {
              output = 0.0;
          }
      }

      m_iq_set = output * m_conf->lo_current_max;
  }

  void stop_pwm_hw(void) {
      TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
      TIM_CCxCmd(    TIM1, TIM_Channel_1, TIM_CCx_Enable);
      TIM_CCxNCmd(   TIM1, TIM_Channel_1, TIM_CCxN_Disable);

      TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
      TIM_CCxCmd(    TIM1, TIM_Channel_2, TIM_CCx_Enable);
      TIM_CCxNCmd(   TIM1, TIM_Channel_2, TIM_CCxN_Disable);

      TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
      TIM_CCxCmd(    TIM1, TIM_Channel_3, TIM_CCx_Enable);
      TIM_CCxNCmd(   TIM1, TIM_Channel_3, TIM_CCxN_Disable);

      TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

  #ifdef HW_HAS_DRV8313
      DISABLE_BR();
  #endif
      m_output_on = false;
  }

  void start_pwm_hw(void) {
      TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
      TIM_CCxCmd(    TIM1, TIM_Channel_1, TIM_CCx_Enable);
      TIM_CCxNCmd(   TIM1, TIM_Channel_1, TIM_CCxN_Enable);

      TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
      TIM_CCxCmd(    TIM1, TIM_Channel_2, TIM_CCx_Enable);
      TIM_CCxNCmd(   TIM1, TIM_Channel_2, TIM_CCxN_Enable);

      TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
      TIM_CCxCmd(    TIM1, TIM_Channel_3, TIM_CCx_Enable);
      TIM_CCxNCmd(   TIM1, TIM_Channel_3, TIM_CCxN_Enable);

      // Generate COM event in ADC interrupt to get better synchronization
  //	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

  #ifdef HW_HAS_DRV8313
      ENABLE_BR();
  #endif
      m_output_on = true;
  }

  int read_hall(void) {
      int h1_1 = READ_HALL1();
      int h2_1 = READ_HALL2();
      int h3_1 = READ_HALL3();

      int h1_2 = READ_HALL1();
      int h2_2 = READ_HALL2();
      int h3_2 = READ_HALL3();

      int h1_3 = READ_HALL1();
      int h2_3 = READ_HALL2();
      int h3_3 = READ_HALL3();

      return middle_of_3_int(h1_1, h1_2, h1_3) |
            (middle_of_3_int(h2_1, h2_2, h2_3) << 1) |
            (middle_of_3_int(h3_1, h3_2, h3_3) << 2);
  }

  radian_t correct_encoder(radian_t obs_angle, radian_t enc_angle, radians_per_second_t speed) {
#ifdef USE_UNITS
      rpm_t rpm_abs = fabsf(speed);
#else
      float rpm_abs = fabsf(speed / ((2.0 * M_PI) / 60.0));
#endif
      static bool using_encoder = true;

      // Hysteresis 5 % of total speed
      rpm_t hyst = m_conf->foc_sl_erpm * 0.05;
      if (using_encoder) {
          if (rpm_abs > (m_conf->foc_sl_erpm + hyst)) {
              using_encoder = false;
          }
      } else {
          if (rpm_abs < (m_conf->foc_sl_erpm - hyst)) {
              using_encoder = true;
          }
      }

      return using_encoder ? enc_angle : obs_angle;
  }

  radian_t correct_hall(radian_t angle, radians_per_second_t speed, second_t dt) {
      static int ang_hall_int_prev = -1;
#ifdef USE_UNITS
      rpm_t rpm_abs = fabsf(speed);
#else
      rpm_t rpm_abs = fabsf(speed / ((2.0 * M_PI) / 60.0));
#endif
      static bool using_hall = true;

      // Hysteresis 5 % of total speed
      rpm_t hyst = m_conf->foc_sl_erpm * 0.1;
      if (using_hall) {
          if (rpm_abs > (m_conf->foc_sl_erpm + hyst)) {
              using_hall = false;
          }
      } else {
          if (rpm_abs < (m_conf->foc_sl_erpm - hyst)) {
              using_hall = true;
          }
      }

      if (using_hall) {
          int ang_hall_int = m_conf->foc_hall_table[read_hall()];

          // Only override the observer if the hall sensor value is valid.
          if (ang_hall_int < 201) {
              static radian_t ang_hall = 0.0_rad;
              radian_t ang_hall_now = (((float)ang_hall_int / 200.0) * 360.0) * PI_rad / 180.0;

              if (ang_hall_int_prev < 0) {
                  // Previous angle not valid
                  ang_hall_int_prev = ang_hall_int;

                  if (ang_hall_int_prev == -2) {
                      // Before was sensorless, initialize with the provided angle
                      ang_hall = angle;
                  } else {
                      // A boot or error has occurred. Use center of hall sensor angle.
                      ang_hall = ((ang_hall_int / 200.0) * 360.0) * PI_rad / 180.0;
                  }
              } else if (ang_hall_int != ang_hall_int_prev) {
                  // A transition was just made. The angle is in the middle of the new and old angle.
                  int ang_avg = abs(ang_hall_int - ang_hall_int_prev);
                  if (ang_avg < 100) {
                      ang_avg = (ang_hall_int + ang_hall_int_prev) / 2;
                  } else if (ang_avg != 100) {
                      ang_avg = (ang_hall_int + ang_hall_int_prev) / 2 + 100;
                  }
                  ang_avg %= 200;
                  ang_hall = (((float)ang_avg / 200.0) * 360.0) * PI_rad / 180.0;
              }

              ang_hall_int_prev = ang_hall_int;

              if (rpm_abs < 100_rpm) {
                  // Don't interpolate on very low speed, just use the closest hall sensor
                  ang_hall = ang_hall_now;
              } else {
                  // Interpolate
                  auto diff = angle_difference_rad(ang_hall, ang_hall_now);
                  if (fabsf(diff) < ((2.0 * PI_rad) / 12.0)) {
                      // Do interpolation
                      ang_hall += speed * dt;
                  } else {
                      // We are too far away with the interpolation
                      ang_hall -= diff / 100.0;
                  }
              }

              norm_angle_rad(ang_hall);
              angle = ang_hall;
          } else {
              // Invalid hall reading. Don't update angle.
              ang_hall_int_prev = -1;

              // Also allow open loop in order to behave like normal sensorless
              // operation. Then the motor works even if the hall sensor cable
              // gets disconnected (when the sensor spacing is 120 degrees).
              if (m_phase_observer_override && m_state == MC_STATE_RUNNING) {
                  angle = m_phase_now_observer_override;
              }
          }
      } else {
          // We are running sensorless.
          ang_hall_int_prev = -2;
      }

      return angle;
  }
}
