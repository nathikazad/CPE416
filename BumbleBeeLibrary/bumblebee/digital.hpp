/*
 * digital.hpp
 *
 *  Created on: Sep 26, 2015
 *      Author: thomaswillson
 */

#pragma once

enum class BBDigitalDirection
	: uint8_t {
		INPUT = 0, OUTPUT
};

class BBDigital {
public:
	static void init();

	static u08 get_btn(void);
	static bool get_btn_press(void);
	static u08 read(u08 num);
	static void dir(u08 num, BBDigitalDirection dir);
	static void write(u08 num, u08 out);

	static void led(u08 num, u08 state);
	static void led_on(u08 num);
	static void led_off(u08 num);

private:
};

