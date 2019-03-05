/*
 * gpio.c
 *
 *  Created on: 23.02.2019
 *      Author: Bartek
 */
#include "../inc/stm32f103xb.h"
#include "../inc/gpio.h"

void gpio_pin_cfg (GPIO_TypeDef * const port, GpioPin_t pin, GpioMode_t mode) {
	pin = __builtin_ctz(pin) * 4;
	uint32_t volatile * cr_reg;
	uint32_t cr_val;
	cr_reg = &port->CRL;
	if (pin > 28) {
		pin -= 32;
		cr_reg = &port->CRH;
	}
	cr_val = *cr_reg;
	cr_val &= ~((uint32_t)(0x0f << pin));
	cr_val |= (uint32_t)(mode << pin);
	*cr_reg = cr_val;
}
