/*
 Copyright 2012-2016 Benjamin Vedder	benjamin@vedder.se

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

#ifndef HW_ARA_H_
#define HW_ARA_H_

// HW properties
//assert(defined(BOARD_ST_NUCLEO64_F446RE))

#define HW_IS_IHM07M1
//#define HW_IS_IHM08M1

#if defined(HW_IS_IHM07M1) || defined(HW_IS_IHM08M1)
#define HW_IS_IHM0xM1
#endif

#ifdef HW_IS_IHM07M1
	#define HW_NAME			"ARA_F446_IHM07"
	/*
	* Benjamin on March 7, 2015 at 12:04 said:
	* when not using FOC one shunt on the input is actually better than using two
	* on the phases since the software becomes simpler.
	* Also, with only one shunt you can run at 100% duty cycle.
	*/
	#define HW_HAS_3_SHUNTS
	#define HW_HAS_PHASE_SHUNTS // used for FOC only
	#define HW_HAS_DRV8313
#else
	#define HW_NAME			"ARA_F446_IHM08"
//#define HW_HAS_PHASE_SHUNTS // used for FOC only
#endif


#define HW_NO_CCM_RAM           // F446 specific
#define HW_HAS_POTENTIOMETER

// Macros
#define ENABLE_GATE()			palSetPad(GPIOB, 5)
#define DISABLE_GATE()			palClearPad(GPIOB, 5)
#define DCCAL_ON()
#define DCCAL_OFF()

#ifdef HW_IS_IHM0xM1 
	#define IS_DRV_FAULT()			FALSE
#else
	#define IS_DRV_FAULT()			(!palReadPad(GPIOD, 2))
#endif

#define LED_GREEN_ON()			palSetPad(GPIOA, 5)
#define LED_GREEN_OFF()			palClearPad(GPIOA, 5)
#define LED_RED_ON()			palSetPad(GPIOB, 2)
#define LED_RED_OFF()			palClearPad(GPIOB, 2)

#ifdef HW_HAS_DRV8313
	// For power stages with enable pins (e.g. DRV8313)
	#define ENABLE_BR1()			palSetPad(GPIOC, 10)
	#define ENABLE_BR2()			palSetPad(GPIOC, 11)
	#define ENABLE_BR3()			palSetPad(GPIOC, 12)
	#define DISABLE_BR1()			palClearPad(GPIOC, 10)
	#define DISABLE_BR2()			palClearPad(GPIOC, 11)
	#define DISABLE_BR3()			palClearPad(GPIOC, 12)
	#define ENABLE_BR()				palWriteGroup(GPIOC, PAL_GROUP_MASK(3), 10, 7)
	#define DISABLE_BR()			palWriteGroup(GPIOC, PAL_GROUP_MASK(3), 10, 0)

	#define INIT_BR()				palSetPadMode(GPIOC, 10, \
									PAL_MODE_OUTPUT_PUSHPULL | \
									PAL_STM32_OSPEED_HIGHEST); \
									palSetPadMode(GPIOC, 11, \
									PAL_MODE_OUTPUT_PUSHPULL | \
									PAL_STM32_OSPEED_HIGHEST); \
									palSetPadMode(GPIOC, 12, \
									PAL_MODE_OUTPUT_PUSHPULL | \
									PAL_STM32_OSPEED_HIGHEST); \
									DISABLE_BR();
#endif
/*
 * ADC Vector
 *      VESC                IHM07    IHM08
 * 0:	IN0		SENS1/3     IN13     IN13
 * 1:	IN1		SENS2       IN8      IN14/ADC12
 * 2:	IN2		SENS3/1     IN7      IN15/ADC12
 * 3:	IN10	CURR1       IN0
 * 4:	IN11	CURR2       IN11
 * 5:	IN12	CURR3       IN10
 * 6:	IN5		ADC_EXT1
 * 7:	IN6		ADC_EXT2
 * 8:	IN3		TEMP_PCB    IN12
 * 9:	IN14	TEMP_MOTOR  IN14               // not read, no temp probe on chip presently
 * 10:	IN9	    ADC_EXT3    IN9      IN4/ADC12 // potentiometer
 * 11:	IN13	AN_IN       IN1                // VBUS sens
 * 12:	Vrefint
 * 13:	IN0		SENS1       IN13
 * 14:	IN1		SENS2       IN8
 */
#define HW_ADC_CHANNELS			15
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5

// ADC Indexes
#define ADC_IND_SENS1			2
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			0
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_POT             10
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12

// ADC macros and settings

#define V_REG					3.3
#define VIN_R1					169.000 // kOhm
#define VIN_R2					9.310 // kOhm
#ifdef HW_IS_IHM07M1
    #define CURRENT_AMP_GAIN		1.528
#elif defined(HW_IS_IHM08M1)
    #define CURRENT_AMP_GAIN		5.18
#endif

#ifdef HW_IS_IHM07M1
    #define CURRENT_SHUNT_RES		0.33
#elif defined(HW_IS_IHM08M1)
    #define CURRENT_SHUNT_RES		0.01
#endif

#define ADC_RES 4095.0

// Voltage on ADC channel
#define ADC_TO_VOLTS(adc_val)   ((float)(adc_val) / ADC_RES * V_REG)
#define VOLTS_TO_ADC(volts)     ((int16_t)((volts)/ V_REG   * ADC_RES ))
#define ADC_VOLTS(ch)           ADC_TO_VOLTS(ADC_Value[ch])

// Input voltage
#define VOLTAGE_DIVIDER        ((VIN_R1 + VIN_R2) / VIN_R2)
#define GET_INPUT_VOLTAGE()	   (ADC_VOLTS(ADC_IND_VIN_SENS) * VOLTAGE_DIVIDER)

// BEMF Voltage
#define R39_IHM0X 10.0 // 10k ohms
#define R36_IHM0X 2.2  // 2.2k ohms
#define V_D_IHM0X 0.3  // schotky BAT30kFILM typical voltage drop
#define PHASE_DIVIDER ((R39_IHM0X + R36_IHM0X) / R36_IHM0X)

// converts straight adc reading to BEMF voltage
#ifdef HW_IS_IHM0xM1
#define SCALE_V_P(V)    (V * VOLTAGE_DIVIDER / PHASE_DIVIDER)
#define CONV_ADC_V(V)   (((V) * VOLTAGE_DIVIDER + V_D_IHM0X * ADC_RES / V_REG * (PHASE_DIVIDER-1)) / PHASE_DIVIDER)
#else
#define SCALE_V_P(V)    (V)
#define CONV_ADC_V(V)   (V)
#endif

#define GET_BEMF_VOLTAGE(adc_val) ((ADC_TO_VOLTS(adc_val) - V_D_IHM0X ) * PHASE_DIVIDER + V_D_IHM0X)
#define GET_BEMF_VOLTAGE_CH(adc_ch) (GET_BEMF_VOLTAGE(ADC_Value[adc_ch]))

// NTC Termistors
#define NTC_CONV(val)           (10 * val / (5.3 * val + 4.7))
#define NTC_RES_2               4700.0
#define NTC_RES(adc_val)		((ADC_RES * NTC_RES_2) / adc_val - NTC_RES_2)
#define NTC_BETA_TEMP           3380.0
#define NTC_REF_RES             10000.0 // resistor value at NTC_REF_TEMP deg
#define NTC_REF_TEMP            298.15 // 25 deg
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / NTC_REF_RES) / NTC_BETA_TEMP) + (1.0 / NTC_REF_TEMP)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((ADC_RES / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)    (1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
	#define CURR1_DOUBLE_SAMPLE		FALSE
#endif
#ifndef CURR2_DOUBLE_SAMPLE
	#define CURR2_DOUBLE_SAMPLE		FALSE
#endif
#ifndef CURR3_DOUBLE_SAMPLE
	#define CURR3_DOUBLE_SAMPLE		FALSE
#endif

// Number of servo outputs
#define HW_SERVO_NUM			2

// UART Peripheral
#define HW_UART_DEV				UARTD6
#define HW_UART_GPIO_AF			GPIO_AF_USART6
#define HW_UART_TX_PORT			GPIOC
#define HW_UART_TX_PIN			6
#define HW_UART_RX_PORT			GPIOC
#define HW_UART_RX_PIN			7

// ICU Peripheral for servo decoding
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOA
#define HW_HALL_ENC_PIN1		15
#define HW_HALL_ENC_GPIO2		GPIOB
#define HW_HALL_ENC_PIN2		3
#define HW_HALL_ENC_GPIO3		GPIOB
#define HW_HALL_ENC_PIN3		10
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// NRF pins
#define NRF_PORT_CSN			GPIOB
#define NRF_PIN_CSN				12
#define NRF_PORT_SCK			GPIOB
#define NRF_PIN_SCK				4
#define NRF_PORT_MOSI			GPIOB
#define NRF_PIN_MOSI			3
#define NRF_PORT_MISO			GPIOD
#define NRF_PIN_MISO			2

// SPI pins
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]

#ifdef HW_IS_IHM0xM1
#define ADC_V_ZERO		CONV_ADC_V((ADC_Value[ADC_IND_VIN_SENS] / 2))
#else
#define ADC_V_ZERO		(ADC_Value[ADC_IND_VIN_SENS] / 2)
#endif
// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Default motor configuration values for this HW
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE			MOTOR_TYPE_BLDC
#endif

#ifndef MCCONF_L_CURRENT_MAX
#define MCCONF_L_CURRENT_MAX				2.0	// Current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN				-2.0	// Current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX				2.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN				-2.0	// Input current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT			10.0	// The maximum absolute current above which a fault is generated
#endif

#ifndef MCCONF_CC_MIN_CURRENT
#define MCCONF_CC_MIN_CURRENT				0.01	// Minimum allowed current
#endif

// FOC settings
#ifndef MCCONF_FOC_CURRENT_KP
#define MCCONF_FOC_CURRENT_KP				4.8
#endif
#ifndef MCCONF_FOC_CURRENT_KI
#define MCCONF_FOC_CURRENT_KI				5800.0
#endif
#ifndef MCCONF_FOC_F_SW
#define MCCONF_FOC_F_SW						20000.0
#endif
#ifndef MCCONF_FOC_MOTOR_L
#define MCCONF_FOC_MOTOR_L					0.0048
#endif
#ifndef MCCONF_FOC_MOTOR_R
#define MCCONF_FOC_MOTOR_R					5.8
#endif
#ifndef MCCONF_FOC_MOTOR_FLUX_LINKAGE
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE		0.023
#endif
#ifndef MCCONF_FOC_OBSERVER_GAIN
#define MCCONF_FOC_OBSERVER_GAIN			31e4
#endif
#ifndef MCCONF_FOC_OPENLOOP_RPM
#define MCCONF_FOC_OPENLOOP_RPM				500.0
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_HYST
#define MCCONF_FOC_SL_OPENLOOP_HYST			0.5
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_TIME
#define MCCONF_FOC_SL_OPENLOOP_TIME			0.3
#endif

// Setting limits
#define HW_LIM_CURRENT          -120.0, 120.0
#define HW_LIM_CURRENT_IN       -120.0, 120.0
#define HW_LIM_CURRENT_ABS      0.0, 160.0
#define HW_LIM_VIN              6.0, 57.0
#define HW_LIM_ERPM             -200e3, 200e3
#define HW_LIM_DUTY_MIN         0.0, 0.1
#define HW_LIM_DUTY_MAX         0.0, 0.99
#define HW_LIM_TEMP_FET         celsius_t{-40.0}, celsius_t{110.0}

#endif
