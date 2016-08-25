#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "globals.hpp"
#include "lcd.hpp"

#define _HOME_ADDR      0x80
#define _LINE_INCR      0x40

#define LCD_E_PIN 4  //PC4
#define LCD_RS_PIN 7 //PF7
#define LCD_DATA_PORT PORTA

#ifndef BOOTLOADER

#define SCROLLING 1

#endif

void BBLCD::e_Clk(void) {
	_delay_us(2);
	sbi(PORTC, LCD_E_PIN);
	_delay_us(2);
	cbi(PORTC, LCD_E_PIN);
	_delay_us(2);
}

void BBLCD::write_lcd(u08 data) {
	cli();
	LCD_DATA_PORT = data;
	e_Clk();
	sei();
}

void BBLCD::write_control(u08 data) {
	cbi(PORTF, LCD_RS_PIN); //set RS low
	write_lcd(data);
}

void BBLCD::write_data(u08 data) {
	sbi(PORTF, LCD_RS_PIN); //set RS high
	write_lcd(data);
}

void BBLCD::set_cursor(uint8_t col, uint8_t row) {
	if (col >= 16 || row >= 2) {
		return;
	}

	u08 addr = _HOME_ADDR + row * _LINE_INCR + col;
	write_control(addr);
	_delay_us(37);
}

void BBLCD::init(void) {
	DDRC |= _BV(LCD_E_PIN);
	DDRF |= _BV(LCD_RS_PIN);
	DDRA = 0xFF; //make all the data pins output

//   write_control(0x38);  //function set
	_delay_ms(5);

	write_control(0x38);  //function set
	_delay_us(160);

	write_control(0x38);  //function set
	_delay_us(160);
	write_control(0x38);  //function set
	_delay_us(160);
	write_control(0x08);  //turn display off
	_delay_us(160);
	write_control(0x01);  //clear display
	_delay_us(4000);
	write_control(0x06);  //set entry mode
	_delay_us(160);  //*/

	write_control(0x38); //function set
	_delay_us(100);
	write_control(0x0C);
	_delay_us(100);

}

void BBLCD::print(const char* string) {
	u08 i = 0;

	while (string[i] != 0) {
		write_data(string[i]);
		_delay_us(160);
		i++;
	}
}

void BBLCD::print(u16 number) {
	u08 i;
	u16 base = 9999;
	u08 leading = 1;
	u08 digit;

	if (number == 0) {
		write_data(48);
		_delay_us(160);
		return;
	}

	for (i = 0; i < 5; i++) {
		digit = number / (base + 1);

		if (digit != 0)
			leading = 0;

		if (number > base) {
			if (!leading) {
				write_data(48 + digit);
				_delay_us(160);
			}
			number = number % (base + 1);
		} else {
			if (!leading) {
				write_data(48 + digit);
				_delay_us(160);
			}
		}

		base = base / 10;
	}
}

void BBLCD::printf(const char *fmt, ...) {
	char s[64];
	va_list args;
	va_start(args, fmt);

	vsnprintf(s, 64, fmt, args);
	print(s);
}

void BBLCD::clear(void) {
	write_control(0x01);  //clear display
	_delay_us(4000);
}

void BBLCD::scroll_message(const char* message, int length, int iteration) {
	set_cursor(0, 0);
	print("          ");
	set_cursor(0, 0);
	print(message + iteration);
}
