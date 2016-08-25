/*
 * servo.hpp
 *
 *  Created on: Sep 26, 2015
 *      Author: thomaswillson
 */

#pragma once

#define SERVO0_PIN 0 //PC0
#define SERVO1_PIN 1 //PC1
#define SERVO2_PIN 2 //PC2
#define SERVO3_PIN 3 //PF3

class BBServo {
public:
	BBServo(u08 servoNumber);

	static void init();
	inline static void ISR_Routine() {
		if (state & 1) {
			OCR1A += low_time[state >> 1];
			PORTC &= ~_BV(state >> 1);
		} else {
			OCR1A += high_time[state >> 1];
			PORTC |= _BV(state >> 1);
		}

		state += 1;
		state &= 7;
	}

	inline void set_position(u08 position) {
		high_time[servoNumber] = 2500 + 10 * position;
		low_time[servoNumber] = 12500 - high_time[servoNumber];
	}

	inline void set_speed(int speed) {
		if(speed>100)
			speed=100;
		else if(speed<-100)
			speed=-100;
		set_position((float) speed * .4 + 127);
	}

private:
	static const u08 numServos = 4;
	static u08 state;
	static u16 high_time[numServos];
	static u16 low_time[numServos];

	u08 servoNumber;
};

