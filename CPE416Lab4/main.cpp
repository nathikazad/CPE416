/**
 * Name:  Thomas Willson & Nathik Salam
 * Assignment number Lab 1 parts 1-4 (Change Section # to compile different versions.)
 * Description: Implements a LED Pulser, Displays two names and flips between them with a
 * 	button press, has a simple pong game, and moves numbers around on the screen responding
 * 	to tilt.
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <bumblebee/globals.hpp>

const float leftMotorGain = 1.0;
const float rightMotorGain = 1.0;
//BBServo leftMotor(1);
//BBServo rightMotor(0);


void init_encoder() {
	// enable encoder interrupts
	EIMSK = 0;
	EIMSK |= _BV(PCIE1) | _BV(PCIE0);
	PCMSK1 |= _BV(PCINT13); //PB5 - digital 5
	PCMSK0 |= _BV(PCINT6);  //PE6 - digital 4
	// enable pullups
	PORTE |= _BV(PE6);
	PORTB |= _BV(PB5);
}
volatile uint16_t left_encoder = 0;
volatile uint16_t right_encoder = 0;

ISR(PCINT0_vect) {
	left_encoder++;  //increment left encoder
}
ISR(PCINT1_vect) {
	right_encoder++;  //increment right encoder
}

void drive(float directionChange, float distanceChange) {
	const float acceptableError = 3.0;
	const float directionP = .25, distanceP = .1;
	float directionError = directionChange, distanceError = distanceChange;

	left_encoder = right_encoder = 0;
	BBLCD::set_cursor(0,0);
//	leftMotor.set_speed(50);
//	rightMotor.set_speed(-50);
//	while (directionError > acceptableError || distanceError > acceptableError) {
//		BBLCD::set_cursor(0,0);
//		BBLCD::printf("%d", right_encoder);
//		BBLCD::set_cursor(0,1);
//		BBLCD::printf("%d", left_encoder);
//		float directionCorrection = (directionChange - (left_encoder - right_encoder)) * directionP;
//		float distanceCorrection = (distanceChange - (left_encoder + right_encoder) / 2) * distanceP;
//
//		int leftSpeed = leftMotorGain * (distanceCorrection - directionCorrection);
//		int rightSpeed = -rightMotorGain * (distanceCorrection + directionCorrection);
//	}
}

void driveStraight(float count) {
	drive(0, count);
}

void turn(float count) {
	drive(count, 0);
}

void driveSquare() {
	const float sideCount = 25;
	const float turnCount = 5;

	while (1) {
		driveStraight(sideCount);
		turn(turnCount);
	}
}

void driveBowtie() {
	const float longSide = 500;
	const float shortSide = 200;
	const float turnCount = 250;

	while (1) {
		driveStraight(shortSide);
		turn(turnCount);
		driveStraight(longSide);
		turn(turnCount);
	}
}

int main() {
	BBUtility::init();
	init_encoder();
	int16_t error=0;
	while(1)
	{
//		error=left_encoder-right_encoder;
//		int16_t new_left_speed=-70-error*2;
//		if(new_left_speed>0)
//			new_left_speed=0;
//		else if(new_left_speed<-100)
//			new_left_speed=-100;
//		int16_t new_right_speed=70-error*2;
//		if(new_right_speed<0)
//			new_right_speed=0;
//		else if(new_right_speed>100)
//			new_right_speed=100;
//		leftMotor.set_speed(new_left_speed);
//		rightMotor.set_speed(new_right_speed);
		BBLCD::set_cursor(0,0);
		BBLCD::printf("%d", left_encoder);
		BBLCD::set_cursor(0,1);
		BBLCD::printf("%d", right_encoder);

	}
//	turn(10);
}

