/*
 * lcd.c
 *
 *  Created on: 24.02.2019
 *      Author: Bartek
 */

#include "../inc/stm32f103xb.h"
#include "../inc/lcd.h"
#include "../inc/delay.h"
#include "../inc/gpio.h"

void lcd_write_half(uint8_t data) {
	LCD_E_HIGH;

	if (data & 0x01) LCD_D4_PORT->ODR |= LCD_D4_PIN;
	else LCD_D4_PORT->ODR &= ~LCD_D4_PIN;

	if (data & 0x02) LCD_D5_PORT->ODR |= LCD_D5_PIN;
	else LCD_D5_PORT->ODR &= ~LCD_D5_PIN;

	if (data & 0x04) LCD_D6_PORT->ODR |= LCD_D6_PIN;
	else LCD_D6_PORT->ODR &= ~LCD_D6_PIN;

	if (data & 0x08) LCD_D7_PORT->ODR |= LCD_D7_PIN;
	else LCD_D7_PORT->ODR &= ~LCD_D7_PIN;

	LCD_E_LOW;
}

void lcd_write_byte(uint8_t data) {
	lcd_write_half(data >> 4);
	lcd_write_half(data);
	delay_10us(12);
}

void lcd_write_cmd(uint8_t cmd) {
	LCD_RS_LOW;
	lcd_write_byte(cmd);
}

void lcd_write_char(char data) {
	LCD_RS_HIGH;
	lcd_write_byte(data);
}

void lcd_locate(uint8_t x, uint8_t y) {

	switch(y) {
		case 0:
			lcd_write_cmd( LCD_SET_DDRAM | (LCD_LINE1 + x) );
			break;

		case 1:
			lcd_write_cmd( LCD_SET_DDRAM | (LCD_LINE2 + x) );
			break;
	}

}

void lcd_gpio_init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	gpio_pin_cfg(LCD_D4_PORT, LCD_D4_PIN, gpio_mode_output_PP_50MHz);
	gpio_pin_cfg(LCD_D5_PORT, LCD_D5_PIN, gpio_mode_output_PP_50MHz);
	gpio_pin_cfg(LCD_D6_PORT, LCD_D6_PIN, gpio_mode_output_PP_50MHz);
	gpio_pin_cfg(LCD_D7_PORT, LCD_D7_PIN, gpio_mode_output_PP_50MHz);
	gpio_pin_cfg(LCD_RS_PORT, LCD_RS_PIN, gpio_mode_output_PP_50MHz);
	gpio_pin_cfg(LCD_E_PORT, LCD_E_PIN, gpio_mode_output_PP_50MHz);
}

void LCD_Init(void) {
	lcd_gpio_init();

//	delay_10us(10000);
//
//	LCD_E_LOW;
//	LCD_RS_LOW;
//
//	lcd_write_half(0b11);
//	delay_10us(500);
//	lcd_write_half(0b11);
//	delay_10us(10);
//	lcd_write_half(0b11);
//	delay_10us(10);
//	lcd_write_half(0b10);
//	delay_10us(10);
//
//	lcd_write_cmd(0b00101000);
//	delay_10us(10);
//	lcd_write_cmd(0b00001000);
//	delay_10us(10);
//	lcd_write_cmd(0b00000001);
//	delay_10us(500);
//	lcd_write_cmd(0b00000110);
//	delay_10us(10);
//	lcd_write_cmd(0b00001100);
//	delay_10us(10);


	delay_10us(1500);

	LCD_E_LOW;
	LCD_RS_LOW;

	lcd_write_half(0x03);
	delay_10us(410);
	lcd_write_half(0x03);
	delay_10us(10);
	lcd_write_half(0x03);
	delay_10us(10);
	lcd_write_half(0x02);
	delay_10us(10);

	lcd_write_cmd( LCD_FUNC | LCD_4_BIT | LCD_TWO_LINE | LCD_FONT_5x7 );
	lcd_write_cmd( LCD_ONOFF | LCD_DISP_ON );
	lcd_write_cmd( LCD_CLEAR );
	delay_10us(500);
	lcd_write_cmd( LCD_ENTRY_MODE | LCD_EM_SHIFT_CURSOR | LCD_EM_RIGHT );

}

void lcd_write_str(char *text) {
	while(*text)
		lcd_write_char(*text++);

}


void lcd_str_XY(uint8_t x, uint8_t y, char * text) {
	lcd_locate(x,y);

	while(*text)
		lcd_write_char(*text++);

}
