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

#include "hw.h"
#ifdef HW_VERSION_ARA

#include "ch.h"
#include "hal.h"
#include "utils.h"

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

#ifdef HW_IS_IHM0xM1
    // Nucleo Blue Button
    palSetPadMode(GPIOC, GPIOC_BUTTON,
                  PAL_MODE_INPUT_PULLUP);
#endif

	// LED green
	palSetPadMode(GPIOA, 5,
                  PAL_MODE_OUTPUT_PUSHPULL |
                  PAL_STM32_OSPEED_HIGHEST);
	// LED red
	palSetPadMode(GPIOB, 2,
                  PAL_MODE_OUTPUT_PUSHPULL |
                  PAL_STM32_OSPEED_HIGHEST);

#ifdef HW_IS_IHM0xM1
    // For IHM0xM1 GPIO BEMF sensing:
    palSetPadMode(GPIOC, 9,
            PAL_MODE_OUTPUT_PUSHPULL |
            PAL_STM32_OSPEED_HIGHEST);
    // set to GND for IHM0x voltage sensing (see schematics in user manual)
    palClearPad(GPIOC, 9);
#endif


#ifndef HW_IS_IHM0xM1
	// ENABLE_GATE
	palSetPadMode(GPIOB, 5,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	ENABLE_GATE();
#endif

	// Motor PWM configuration. The DRV8313 has three enable pins and 3 pwm pins.
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);

#ifdef HW_HAS_DRV8313
	INIT_BR();
#elif defined(HW_IS_IHM08M1)
	// Motor PWM configuration
    palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUPDR_FLOATING);
    palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUPDR_FLOATING);
    palSetPadMode(GPIOB, 1, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUPDR_FLOATING);
#endif

	// Hall sensors
#ifdef USE_HALL
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);
#endif

#ifndef HW_IS_IHM0xM1
	// Fault pin
	palSetPadMode(GPIOD, 2, PAL_MODE_INPUT_PULLUP);
#endif

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); // CURRENT1
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG); // VBUS
	//palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	//palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	//palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	//palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG); // CURRENT3
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG); // CURRENT2
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG); // ADC_TEMP
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG); // BEMF1

#ifdef HW_IS_IHM07M1
	// Strange: PC4 is not used but commenting the line below
	// causes some sort of fault (blinking red light on the IHM and motor doesn't run
    palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG); // not assigned?

    palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG); // potentiometer
    palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG); // BEMF2
    palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_ANALOG); // BEMF3
#elif defined(HW_IS_IHM08M1)
    palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG); // potentiometer
    palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG); // BEMF2
    palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG); // BEMF3
#endif
}

#ifdef HW_IS_IHM07M1
#define ADC_CHAN_SENS1 ADC_Channel_13
#define ADC_CHAN_SENS2 ADC_Channel_8
#define ADC_CHAN_SENS3 ADC_Channel_7
#define ADC_CHAN_POT   ADC_Channel_9
#elif defined(HW_IS_IHM08M1)
#define ADC_CHAN_SENS1 ADC_Channel_13
#define ADC_CHAN_SENS2 ADC_Channel_14
#define ADC_CHAN_SENS3 ADC_Channel_15
#define ADC_CHAN_POT   ADC_Channel_4
#endif

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_CHAN_SENS3,      1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0,       2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5,       3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14,      4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_CHAN_SENS2, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6,  3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_CHAN_POT,   4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_CHAN_SENS2, 5, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_CHAN_SENS1, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1,  4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_CHAN_SENS1, 5, ADC_SampleTime_15Cycles);

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0,  1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0,  2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0,  3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

#endif
