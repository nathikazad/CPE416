/*
 * digital.cpp
 *
 *  Created on: Sep 26, 2015
 *      Author: thomaswillson
 */

#include <avr/io.h>

#include "globals.hpp"

#define DIGITAL0_PIN 0 //PE0
#define DIGITAL1_PIN 1 //PE1
#define DIGITAL2_PIN 3 //PG3
#define DIGITAL3_PIN 4 //PG4
#define DIGITAL4_PIN 6 //PE6
#define DIGITAL5_PIN 5 //PB5
#define DIGITAL6_PIN 6 //PB6
#define DIGITAL7_PIN 5 //PC5
#define DIGITAL8_PIN 6 //PC6
#define DIGITAL9_PIN 7 //PC7
#define DIGITAL10_PIN 0 //PB0/SS
#define DIGITAL11_PIN 2 //PB2/MOSI
#define DIGITAL12_PIN 3 //PB3/MISO
#define DIGITAL13_PIN 1 //PB1/SCK

#define SW1_PIN 7  //PE7
#define LED0_PIN 0 //PG0
#define LED1_PIN 1 //PG1

void BBDigital::init() {
	//make LED0 and LED1 outputs
	DDRG |= _BV(LED1_PIN) | _BV(LED0_PIN);

	//enable SW1 pull
	sbi(PORTE, SW1_PIN);
}

u08 BBDigital::get_btn(void) {
	if (PINE & _BV(SW1_PIN))
		return 0;
	return 1;
}

bool BBDigital::get_btn_press(void) {
	static bool buttonLatch = false;
	bool result = false;

	if (get_btn() && !buttonLatch) {
		result = true;
		buttonLatch = true;
	} else if (!get_btn() && buttonLatch) {
		buttonLatch = false;
	}

	return result;
}

u08 BBDigital::read(u08 num) {
	switch (num) {
	case 0:
	case 1:
		if (PINE & _BV(num))
			return 1;
		break;
	case 2:
	case 3:
		if (PING & _BV(num + 1))
			return 1;
		break;
	case 4:
		if (PINE & _BV(6))
			return 1;
		break;
	case 5:
	case 6:
		if (PINB & _BV(num))
			return 1;
		break;
	case 7:
	case 8:
	case 9:
		if (PINC & _BV(num - 2))
			return 1;
		break;
	case 10:
		if (PINB & _BV(0))
			return 1;
		break;
	case 11:
	case 12:
		if (PINB & _BV(num - 9))
			return 1;
		break;
	case 13:
		if (PINB & _BV(1))
			return 1;
		break;
	}

	return 0;
}

#ifndef BOOTLOADER

//set the direction for a digital pin
void BBDigital::dir(u08 num, BBDigitalDirection dir) {
	switch (num) {
	case 0:
	case 1:
		if (dir == BBDigitalDirection::OUTPUT) {
			DDRE |= _BV(num);
		} else {
			DDRE &= ~_BV(num);
		}
		break;
	case 2:
	case 3:
		if (dir == BBDigitalDirection::OUTPUT) {
			DDRG |= _BV(num + 1);
		} else {
			DDRG &= ~_BV(num + 1);
		}
		break;
	case 4:
		if (dir == BBDigitalDirection::OUTPUT) {
			DDRE |= _BV(6);
		} else {
			DDRE &= ~_BV(6);
		}
		break;
	case 5:
	case 6:
		if (dir == BBDigitalDirection::OUTPUT) {
			DDRB |= _BV(num);
		} else {
			DDRB &= ~_BV(num);
		}
		break;
	case 7:
	case 8:
	case 9:
		if (dir == BBDigitalDirection::OUTPUT) {
			DDRC |= _BV(num - 2);
		} else {
			DDRC &= ~_BV(num - 2);
		}
		break;
	case 10:
		if (dir == BBDigitalDirection::OUTPUT) {
			DDRB |= _BV(0);
		} else {
			DDRB &= ~_BV(0);
		}
		break;
	case 11:
	case 12:
		if (dir == BBDigitalDirection::OUTPUT) {
			DDRB |= _BV(num - 9);
		} else {
			DDRB &= ~_BV(num - 9);
		}
		break;
	case 13:
		if (dir == BBDigitalDirection::OUTPUT) {
			DDRB |= _BV(1);
		} else {
			DDRB &= ~_BV(1);
		}
		break;
	}

	return;
}

//set the output value for a digital pin
void BBDigital::write(u08 num, u08 out) {
	switch (num) {
	case 0:
	case 1:
		if (out) {
			PORTE |= _BV(num);
		} else {
			PORTE &= ~_BV(num);
		}
		break;
	case 2:
	case 3:
		if (out) {
			PORTG |= _BV(num + 1);
		} else {
			PORTG &= ~_BV(num + 1);
		}
		break;
	case 4:
		if (out) {
			PORTE |= _BV(6);
		} else {
			PORTE &= ~_BV(6);
		}
		break;
	case 5:
	case 6:
		if (out) {
			PORTB |= _BV(num);
		} else {
			PORTB &= ~_BV(num);
		}
		break;
	case 7:
	case 8:
	case 9:
		if (out) {
			PORTC |= _BV(num - 2);
		} else {
			PORTC &= ~_BV(num - 2);
		}
		break;
	case 10:
		if (out) {
			PORTB |= _BV(0);
		} else {
			PORTB &= ~_BV(0);
		}
		break;
	case 11:
	case 12:
		if (out) {
			PORTB |= _BV(num - 9);
		} else {
			PORTB &= ~_BV(num - 9);
		}
		break;
	case 13:
		if (out) {
			PORTB |= _BV(1);
		} else {
			PORTB &= ~_BV(1);
		}
		break;
	}

	return;
}

#endif

void BBDigital::led(u08 num, u08 state) {
	if (num == 0) {
		if (state == 1)
			sbi(PORTG, LED0_PIN);
		else
			cbi(PORTG, LED0_PIN);
	} else {
		if (state == 1)
			sbi(PORTG, LED1_PIN);
		else
			cbi(PORTG, LED1_PIN);
	}
}

void BBDigital::led_on(u08 num) {
	if (num == 0) {
		sbi(PORTG, LED0_PIN);
	} else {
		sbi(PORTG, LED1_PIN);
	}
}

void BBDigital::led_off(u08 num) {
	if (num == 0) {
		cbi(PORTG, LED0_PIN);
	} else {
		cbi(PORTG, LED1_PIN);
	}
}

