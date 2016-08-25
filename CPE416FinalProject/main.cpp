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
#include <math.h>
#include <bumblebee/uart.hpp>

#include <bumblebee/globals.hpp>

BBServo leftMotor(1);
BBServo rightMotor(0);

const uint16_t MAX_BUCKETS = 500;

const uint8_t NUM_TICKS_PER_SAMPLE = 1;
const uint8_t NUM_SAMPLES_PER_BUCKET = 2;
const uint8_t BUCKET_INDEX_DIVIDER = NUM_TICKS_PER_SAMPLE * NUM_SAMPLES_PER_BUCKET;

typedef struct MoveRobotResult {
	uint16_t ticksMoved;
	bool lineDetected;

	uint16_t buckets[MAX_BUCKETS];
	uint16_t numBuckets;
	bool shortcutMovement;
} MoveRobotResult;

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
volatile uint16_t encoder = 0;

ISR(PCINT0_vect) {
	encoder++;  //increment encoder
}
ISR(PCINT1_vect) {
	encoder++;  //increment encoder
}

bool isBlockPresent() {
	const uint8_t maxBlockDistanceThreshold = 100;
	const uint8_t distanceSensorPin = 5;

	return BBADC::read(distanceSensorPin) > maxBlockDistanceThreshold;
}

inline float coerce_speed(float speed) {
	if (speed > 100) {
		return 100;
	} else if (speed < -100) {
		return -100;
	}
	return speed;
}

inline float readLineFollower(uint8_t adcInput, float zeroPoint, float multiplier) {
	float adcReading = BBADC::read(adcInput) * multiplier - zeroPoint;

	if (adcReading < 0) {
		return 0;
	}

	return adcReading;
}

bool checkLineFollowers() {
	const int RIGHT_LINE_FOLLOWER_PIN = 0;
	const float RIGHT_LINE_FOLLOWER_ZERO = 0;
	const float RIGHT_LINE_FOLLOWER_GAIN = 1.0;

	const int LEFT_LINE_FOLLOWER_PIN = 1;
	const float LEFT_LINE_FOLLOWER_ZERO = 0;
	const float LEFT_LINE_FOLLOWER_GAIN = 1.2;

	const float LINE_DETECTION_THRESHOLD = 205.0;

	return readLineFollower(RIGHT_LINE_FOLLOWER_PIN, RIGHT_LINE_FOLLOWER_ZERO, RIGHT_LINE_FOLLOWER_GAIN) < LINE_DETECTION_THRESHOLD
			&& readLineFollower(LEFT_LINE_FOLLOWER_PIN, LEFT_LINE_FOLLOWER_ZERO, LEFT_LINE_FOLLOWER_GAIN) < LINE_DETECTION_THRESHOLD;

}

void moveRobot(uint16_t targetTickCount, bool turn, bool reverse, MoveRobotResult *moveResult, bool accurateMovement, bool allowShortcut) {

	const float LEFT_MOTOR_GAIN = 1.0;
	const float RIGHT_MOTOR_GAIN = 1.0;
	const float MAX_BASE_SPEED = 100.0;

	const float CONTROLLER_P = 10.0;

	moveResult->numBuckets = ceil((float) targetTickCount / (float) (BUCKET_INDEX_DIVIDER));
	moveResult->shortcutMovement = false;

	for (uint16_t i = 0; i < moveResult->numBuckets; ++i) {
		moveResult->buckets[i] = 0;
	}

	const uint8_t RANGE_FINDER_PIN = 5;
	const uint16_t SHORTCUT_THRESHOLD = 250;

	encoder = 0;
	uint16_t prevTickCount = encoder;

	while ((!targetTickCount || targetTickCount - encoder > 0) && (reverse || checkLineFollowers())) {
		uint16_t tickCount = encoder;

		if (tickCount != prevTickCount && tickCount % NUM_TICKS_PER_SAMPLE == 0 && tickCount / BUCKET_INDEX_DIVIDER < MAX_BUCKETS) {
			moveResult->buckets[tickCount / BUCKET_INDEX_DIVIDER] += BBADC::read(RANGE_FINDER_PIN);

			if (!reverse && moveResult->buckets[tickCount / BUCKET_INDEX_DIVIDER] > SHORTCUT_THRESHOLD && allowShortcut) {
				moveResult->shortcutMovement = true;
				leftMotor.set_speed(0);
				rightMotor.set_speed(0);
				return;
			}
		}

		prevTickCount = tickCount;

		float controllerSpeed;

		if (targetTickCount > 0) {
			controllerSpeed = CONTROLLER_P * (targetTickCount - tickCount);
		} else {
			controllerSpeed = MAX_BASE_SPEED;
		}

		if (controllerSpeed > MAX_BASE_SPEED) {
			controllerSpeed = MAX_BASE_SPEED;
		}

		float leftSpeed = LEFT_MOTOR_GAIN * controllerSpeed;
		float rightSpeed = RIGHT_MOTOR_GAIN * controllerSpeed;

		if (turn) {
			if (!reverse) {
				leftSpeed = -leftSpeed;
			} else {
				rightSpeed = -rightSpeed;
			}
		} else if (reverse) {
			leftSpeed = -leftSpeed;
			rightSpeed = -rightSpeed;
		}

		leftMotor.set_speed(-leftSpeed);
		rightMotor.set_speed(rightSpeed);
	}

	leftMotor.set_speed(0);
	rightMotor.set_speed(0);

	if (accurateMovement) {
		_delay_ms(400);
	}

	moveResult->ticksMoved = encoder;
}

void turnToPeak(MoveRobotResult *moveResult, bool direction) {
	uint16_t maxBucketContents = 0;
	uint16_t maxBucketIndex = MAX_BUCKETS;

	for (uint16_t i = 0; i < moveResult->numBuckets; ++i) {
//		BBLCD::clear();
//		BBLCD::printf("%1d:%4u", i, moveResult->buckets[i]);
//
//		while (1) {
//			if (BBDigital::get_btn_press()) {
//				_delay_ms(500);
//				break;
//			}
//		}
		if (moveResult->buckets[i] > maxBucketContents) {
			maxBucketContents = moveResult->buckets[i];
			maxBucketIndex = i;
		}
	}

	if (maxBucketIndex == MAX_BUCKETS) { //Go to first
		maxBucketIndex = 0;
	}

	BBLCD::clear();
	BBLCD::printf("T: %3d", maxBucketIndex);
	BBLCD::set_cursor(0, 1);
	BBLCD::printf("V: %4u", maxBucketContents);

	uint16_t ticksFromStartingPoint = (maxBucketIndex - 1) * (BUCKET_INDEX_DIVIDER) + BUCKET_INDEX_DIVIDER / 2;
	uint16_t ticksToMove = abs(moveResult->ticksMoved - ticksFromStartingPoint);

	moveRobot(ticksToMove, true, !direction, moveResult, false, false);
}

int main() {
	BBUtility::init();
	leftMotor.set_speed(0);
	rightMotor.set_speed(0);
	init_encoder();

	BBLCD::clear();
	BBLCD::set_cursor(0, 0);
	BBLCD::printf("Ready!");
	BBLCD::set_cursor(0, 1);
	BBLCD::printf("");

	srand(BBADC::read(5));

	MoveRobotResult moveResult;

//	BBLCD::clear();

	const int RIGHT_LINE_FOLLOWER_PIN = 0;
	const float RIGHT_LINE_FOLLOWER_ZERO = 0;
	const float RIGHT_LINE_FOLLOWER_GAIN = 1.0;

	const int LEFT_LINE_FOLLOWER_PIN = 1;
	const float LEFT_LINE_FOLLOWER_ZERO = 0;
	const float LEFT_LINE_FOLLOWER_GAIN = 1.0;

	while (1) {
//		BBLCD::set_cursor(0, 0);
//		BBLCD::printf("L:%3.0f", readLineFollower(LEFT_LINE_FOLLOWER_PIN, LEFT_LINE_FOLLOWER_ZERO, LEFT_LINE_FOLLOWER_GAIN));
//		BBLCD::set_cursor(0, 1);
//		BBLCD::printf("R:%3.0f", readLineFollower(RIGHT_LINE_FOLLOWER_PIN, RIGHT_LINE_FOLLOWER_ZERO, RIGHT_LINE_FOLLOWER_GAIN));
//		_delay_ms(100);
		if (BBDigital::get_btn_press())
			break;
	}

	bool direction = false;
	while (1) {
		//Survey
		moveRobot(70, true, direction, &moveResult, true, false);
		//Face nearest object if robot didn't find one.
		if (!moveResult.shortcutMovement) {
			turnToPeak(&moveResult, direction);
		}
		//Move object until hit edge
		moveRobot(0, false, false, &moveResult, false, false);
		//Back away from edge
		moveRobot(((float) (rand()) / (float) RAND_MAX) * 30 + 20, false, true, &moveResult, false, false);
		direction = !direction;
	}

//	while (1) {
//		BBLCD::clear();
//		BBLCD::set_cursor(0, 0);
//		BBLCD::printf("%4d", BBADC::read(5));
//		_delay_ms(500);
//	}

}

