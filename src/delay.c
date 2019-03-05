/*
 * delay.c
 *
 *  Created on: 24.02.2019
 *      Author: Bartek
 */

#include "../inc/stm32f103xb.h"
#include "../inc/delay.h"

volatile uint32_t ticks;

__attribute__((interrupt)) void SysTick_Handler (void) {
  ticks++;
}

void delay_10us(uint32_t dlyTicks) {
  uint32_t  curTicks;

  curTicks = ticks;
  while ((ticks - curTicks) < dlyTicks);
}
