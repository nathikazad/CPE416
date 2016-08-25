#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "globals.hpp"
#include "servo.hpp"

u08 BBServo::state;
u16 BBServo::high_time[numServos];
u16 BBServo::low_time[numServos];

ISR(TIMER1_COMPA_vect) {
	BBServo::ISR_Routine();
}

BBServo::BBServo(u08 servoNumber) {
	this->servoNumber = servoNumber;
}

void BBServo::init() {
	state = 0;

	for (u08 i = 0; i < numServos; i++) {
		BBServo servo(i);
		servo.set_position(0);
	}

	DDRC |= _BV(SERVO0_PIN) | _BV(SERVO1_PIN) | _BV(SERVO2_PIN)
			| _BV(SERVO3_PIN);
	TCCR1B = 0;
	TCCR1B |= _BV(CS11);
	TIMSK1 |= _BV(OCIE1A);
	sei();
}
