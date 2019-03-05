/*
 * delay.h
 *
 *  Created on: 24.02.2019
 *      Author: Bartek
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "../inc/stm32f103xb.h"

volatile uint32_t ticks;

__attribute__((interrupt)) void SysTick_Handler (void);

void delay_10us(uint32_t dlyTicks);


#endif /* DELAY_H_ */
