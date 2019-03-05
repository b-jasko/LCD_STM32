#include "../inc/stm32f103xb.h"
#include "../inc/gpio.h"
#include "../inc/delay.h"
#include "../inc/lcd.h"

int main(void) {

	SysTick_Config(80);
	LCD_Init();
//	lcd_gpio_init();
	gpio_pin_cfg(GPIOA, P5, gpio_mode_output_PP_50MHz);
	lcd_write_str("test");



	while(1) {
		GPIOA->ODR |= P5;
		delay_10us(100000);
		GPIOA->ODR &= ~P5;
		delay_10us(100000);
	}
}
