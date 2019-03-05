/*
 * gpio.h
 *
 *  Created on: 23.02.2019
 *      Author: Bartek
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f103xb.h"

typedef enum {
	/* Push-Pull */
	gpio_mode_output_PP_2MHz = 2,
	gpio_mode_output_PP_10MHz = 1,
	gpio_mode_output_PP_50MHz = 3,
	/* Open-Drain */
	gpio_mode_output_OD_2MHz = 6,
	gpio_mode_output_OD_10MHz = 5,
	gpio_mode_output_OD_50MHz = 7,
	/* Push-Pull */
	gpio_mode_alternate_PP_2MHz = 10,
	gpio_mode_alternate_PP_10MHz = 9,
	gpio_mode_alternate_PP_50MHz = 11,
	/* Open-Drain */
	gpio_mode_alternate_OD_2MHz = 14,
	gpio_mode_alternate_OD_10MHz = 13,
	gpio_mode_alternate_OD_50MHz = 15,
	/* Analog input (ADC) */
	gpio_mode_input_analog = 0,
	/* Floating digital input. */
	gpio_mode_input_floating = 4,
	/* Digital input with pull-up/down (depending on the ODR reg.). */
	gpio_mode_input_pull = 8
} GpioMode_t;

typedef enum {
	P0 = 0x00000001,
	P1 = 0x00000002,
	P2 = 0x00000004,
	P3 = 0x00000008,
	P4 = 0x00000010,
	P5 = 0x00000020,
	P6 = 0x00000040,
	P7 = 0x00000080,
	P8 = 0x00000100,
	P9 = 0x00000200,
	P10 = 0x00000400,
	P11 = 0x00000800,
	P12 = 0x00001000,
	P13 = 0x00002000,
	P14 = 0x00004000,
	P15 = 0x00008000
}GpioPin_t;

void gpio_pin_cfg(GPIO_TypeDef * const port, GpioPin_t pin, GpioMode_t mode);

#endif /* GPIO_H_ */
