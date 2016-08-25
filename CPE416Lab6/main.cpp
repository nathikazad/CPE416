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

typedef struct Particle {
	float location;
} Particle;

typedef struct Block {
	float startLocation;
	float endLocation;
} Block;

typedef struct StatsResult {
	float mean;
	float variance;
} StatsResult;

//Configuration
const uint8_t NUMBER_OF_PARTICLES = 100;
const uint8_t NUMBER_OF_PARTICLES_KEPT = 100;
Particle particles[NUMBER_OF_PARTICLES];
const float CIRCUMFERENCE_OF_CIRCLE = 248; //Encoder Counts
const float WIDTH_OF_BLOCK = 20; //Encoder Counts
const float MAX_BASE_SPEED = 60;

//Block Parameters
uint8_t NUMBER_OF_BLOCKS = 0;
uint8_t TARGET_BLOCK = 0;
float blockGivenPositions[5]; //Degrees
Block blocks[5];
const uint16_t NUMBER_OF_PROBABILITY_POINTS = 250;
float blockReadingProbability[NUMBER_OF_PROBABILITY_POINTS];

//Temp Globals
float highestProbability = 0;
uint8_t highestProbIndex = 0;

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

inline float readLight(uint8_t adcInput, float zeroPoint, float multiplier) {
	float adcReading = BBADC::read(adcInput) * multiplier - zeroPoint;

	if (adcReading < 0) {
		return 0;
	}

	return adcReading;
}

inline float trapezoidalDist(float a, float b, float c, float d, float u, float smallP, float x) {
	if (x >= a && x < b) {
		return u * (x - a) / (b - a);
	} else if (x >= b && x < c) {
		return u;
	} else if (x >= c && x < d) {
		return u * (d - x) / (d - c);
	} else {
		return smallP;
	}
}

float probabilityOfReadingBlock(float positionOfParticleAlongBlock) {
	const float a = -3.0, b = 0.0, c = WIDTH_OF_BLOCK, d = WIDTH_OF_BLOCK + 3.0;
	const float u = .8, smallProbability = 0.00;

	return trapezoidalDist(a, b, c, d, u, smallProbability, positionOfParticleAlongBlock);
}

void populateParticles() {
	for (uint8_t i = 0; i < NUMBER_OF_PARTICLES; ++i) {
		particles[i].location = ((float) rand()) / ((float) RAND_MAX) * CIRCUMFERENCE_OF_CIRCLE;
	}
}

void populateBlocks() {
	BBLCD::clear();
	for (uint8_t i = 0; i < NUMBER_OF_BLOCKS; ++i) {
		float degreeLocation = blockGivenPositions[i];
		float centerLocation = degreeLocation / 360.0 * CIRCUMFERENCE_OF_CIRCLE;
		blocks[i].startLocation = centerLocation - WIDTH_OF_BLOCK / 2;
		blocks[i].endLocation = centerLocation + WIDTH_OF_BLOCK / 2;

		if (blocks[i].startLocation < 0) {
			blocks[i].startLocation += CIRCUMFERENCE_OF_CIRCLE;
		}
		if (blocks[i].endLocation > CIRCUMFERENCE_OF_CIRCLE) {
			blocks[i].endLocation -= CIRCUMFERENCE_OF_CIRCLE;
		}
	}
}

void populateBlockReadingProbabilityLUT() {
	const float smallP = 0.025;

	for (uint16_t i = 0; i < NUMBER_OF_PROBABILITY_POINTS; ++i) {
		blockReadingProbability[i] = smallP;
	}

	for (uint8_t i = 0; i < NUMBER_OF_BLOCKS; ++i) {
		float start = blocks[i].startLocation;

		for (uint16_t j = 0; j < NUMBER_OF_PROBABILITY_POINTS; ++j) {
			float x = j * CIRCUMFERENCE_OF_CIRCLE / (float) NUMBER_OF_PROBABILITY_POINTS;
			float relativeLocation = x - start;
			if (relativeLocation < 0) {
				relativeLocation += CIRCUMFERENCE_OF_CIRCLE;
			}
			blockReadingProbability[j] += probabilityOfReadingBlock(relativeLocation);
		}
	}
}

inline float probabilityOfCorrectness(float x, bool blockPresent) {
	uint16_t index = (x / CIRCUMFERENCE_OF_CIRCLE) * (float) NUMBER_OF_PROBABILITY_POINTS;

	return blockPresent ? blockReadingProbability[index] : 1 - blockReadingProbability[index];
}

float findParticleLocationDelta(float actualRobotMovement) {
	const float sigma = 1;
	float random1, random2;

	random1 = (float) rand() / (float) RAND_MAX;
	random2 = (float) rand() / (float) RAND_MAX;

	float z = sqrt(-2 * log(random1)) * cos(2 * M_PI * random2);

	return sigma * z + actualRobotMovement;
}

void moveParticles(float actualRobotMovement) {
	for (uint8_t i = 0; i < NUMBER_OF_PARTICLES; ++i) {
		particles[i].location += findParticleLocationDelta(actualRobotMovement);

		if (particles[i].location > CIRCUMFERENCE_OF_CIRCLE) {
			particles[i].location -= CIRCUMFERENCE_OF_CIRCLE;
		}
	}
}

uint16_t moveRobot(uint16_t targetTickCount) {
	const int rightSensorPin = 0;
	const float rightZero = 0;
	const float rightMult = 1.0;

	const float leftMotorGain = 1.0;
	const float rightMotorGain = 1.0;

	const float directionControllerP = 2.2;
	const float distanceControllerP = 10.0;

	encoder = 0;

	while (targetTickCount - encoder > 0) {
		const float targetLightValue = 210.0;
		float rightLight = readLight(rightSensorPin, rightZero, rightMult);

		float error = targetLightValue - rightLight;
		float correction = error * directionControllerP;

		float baseSpeed = distanceControllerP * (targetTickCount - encoder);

		if (baseSpeed > MAX_BASE_SPEED) {
			baseSpeed = MAX_BASE_SPEED;
		}

		float leftSpeed = leftMotorGain * (baseSpeed + correction);
		float rightSpeed = rightMotorGain * (baseSpeed - correction);

		leftMotor.set_speed(-leftSpeed);
		rightMotor.set_speed(rightSpeed);
	}

	leftMotor.set_speed(0);
	rightMotor.set_speed(0);

	_delay_ms(200);

	return encoder;
}

void redistributeParticles() {
	bool blockPresent = isBlockPresent();
	float cumulativeProbabilities[NUMBER_OF_PARTICLES];
	float sumOfProbabilities = 0;
	highestProbability = 0;

	for (uint8_t i = 0; i < NUMBER_OF_PARTICLES; ++i) {
		float prob = probabilityOfCorrectness(particles[i].location, blockPresent);
		if (prob > highestProbability) {
			highestProbability = prob;
			highestProbIndex = i;
		}
		sumOfProbabilities += prob;
		cumulativeProbabilities[i] = sumOfProbabilities;
	}

	float randomProbability = (float) rand() / (float) RAND_MAX * sumOfProbabilities;
	float probabilityIncrement = sumOfProbabilities / (float) NUMBER_OF_PARTICLES_KEPT;

	uint8_t indexOfSelectedParticle = 0;
	Particle newParticles[NUMBER_OF_PARTICLES];
	for (uint8_t i = 0; i < NUMBER_OF_PARTICLES; ++i) {
		while (cumulativeProbabilities[indexOfSelectedParticle] < randomProbability)
			indexOfSelectedParticle = (indexOfSelectedParticle + 1) % NUMBER_OF_PARTICLES;
		newParticles[i] = particles[indexOfSelectedParticle];
		randomProbability += probabilityIncrement;
		if (randomProbability > sumOfProbabilities) {
			randomProbability -= sumOfProbabilities;
			indexOfSelectedParticle = 0;
		}
	}

	memcpy((void *) particles, newParticles, sizeof(Particle) * NUMBER_OF_PARTICLES_KEPT);

	for (uint8_t i = NUMBER_OF_PARTICLES_KEPT; i < NUMBER_OF_PARTICLES; ++i) {
		particles[i].location = ((float) rand()) / ((float) RAND_MAX) * CIRCUMFERENCE_OF_CIRCLE;
	}
}

StatsResult calculateVariance() {
	StatsResult result;

	float xSum = 0;
	float ySum = 0;

	//Compute average unit vector.
	for (uint8_t i = 0; i < NUMBER_OF_PARTICLES; ++i) {
		float radianLocation = (particles[i].location / CIRCUMFERENCE_OF_CIRCLE) * 2 * M_PI;
		xSum += cos(radianLocation);
		ySum += sin(radianLocation);
	}

	float xMean = xSum / NUMBER_OF_PARTICLES;
	float yMean = ySum / NUMBER_OF_PARTICLES;

	//Convert back to circumference.
	float resultantPercent = atan2(yMean, xMean) / (2 * M_PI);
	if (resultantPercent < 0) {
		resultantPercent = 1 + resultantPercent;
	}
	result.mean = resultantPercent * CIRCUMFERENCE_OF_CIRCLE;

	result.variance = 0;
	for (uint8_t i = 0; i < NUMBER_OF_PARTICLES; ++i) {
		//Convert to cartesian and use distance formula to find variance.
		float radianLocation = (particles[i].location / CIRCUMFERENCE_OF_CIRCLE) * 2 * M_PI;
		float xLoc = cos(radianLocation);
		float yLoc = sin(radianLocation);
		result.variance += sqrt(pow(xLoc - xMean, 2) + pow(yLoc - yMean, 2));
	}

	return result;
}

int main() {
	const uint16_t DISTANCE_TO_MOVE = 5; //Encoder Ticks

	BBUtility::init();
	leftMotor.set_speed(0);
	rightMotor.set_speed(0);
	init_encoder();
	srand(BBADC::read(0));

	BBLCD::clear();
	BBLCD::set_cursor(0, 0);
	BBLCD::printf("# Blocks");
	while (1) {
		uint8_t accel = BBAccelerometer::get_z();
		uint8_t numBlocks = (float) accel / 64.0 * 2.0 + 3.0;

		BBLCD::set_cursor(0, 1);
		BBLCD::printf("%7d", numBlocks);

		if (BBDigital::get_btn_press()) {
			NUMBER_OF_BLOCKS = numBlocks;
			break;
		}
		_delay_ms(250);
	}
	BBLCD::clear();
	BBLCD::set_cursor(0, 0);
	BBLCD::printf("TRGT BLK");
	while (1) {
		uint8_t accel = BBAccelerometer::get_z();
		uint8_t targetBlock = (float) accel / 64.0 * NUMBER_OF_BLOCKS - 1;

		BBLCD::set_cursor(0, 1);
		BBLCD::printf("%7d", targetBlock);

		if (BBDigital::get_btn_press()) {
			TARGET_BLOCK = targetBlock;
			break;
		}
		_delay_ms(250);
	}

	for (int i = 0; i < NUMBER_OF_BLOCKS; ++i) {
		BBLCD::clear();
		BBLCD::set_cursor(0, 0);
		BBLCD::printf("Block %d", i);

		while (1) {
			uint8_t accel = BBAccelerometer::get_z();
			float blockPosition = (float) accel / 64 * 8;

			BBLCD::set_cursor(0, 1);
			BBLCD::printf("%7.2f", round(blockPosition) * 45);

			if (BBDigital::get_btn_press()) {
				blockGivenPositions[i] = round(blockPosition) * 45;
				break;
			}
			_delay_ms(250);
		}
	}

	BBLCD::clear();
	BBLCD::set_cursor(0, 0);
	BBLCD::printf("P 2 S");
	BBLCD::set_cursor(0, 1);
	BBLCD::printf("T: %3d", TARGET_BLOCK);
	while (1) {
		if (BBDigital::get_btn_press()) {
			_delay_ms(1000);
			break;
		}
	}
	while (1) {
		populateParticles();
		populateBlocks();
		populateBlockReadingProbabilityLUT();

		encoder = 0;

		float radianLocation = (blocks[TARGET_BLOCK].startLocation / CIRCUMFERENCE_OF_CIRCLE) * 2 * M_PI;
		float xStart = cos(radianLocation);
		float yStart = sin(radianLocation);

		radianLocation = (blocks[TARGET_BLOCK].endLocation / CIRCUMFERENCE_OF_CIRCLE) * 2 * M_PI;
		float xEnd = cos(radianLocation);
		float yEnd = sin(radianLocation);

		float xMean = (xStart + xEnd) / 2;
		float yMean = (yStart + yEnd) / 2;

		float resultantPercent = atan2(yMean, xMean) / (2 * M_PI);
		if (resultantPercent < 0) {
			resultantPercent = 1 + resultantPercent;
		}
		float targetBlockLocation = resultantPercent * CIRCUMFERENCE_OF_CIRCLE;

		BBLCD::clear();
		BBLCD::set_cursor(0, 0);
		BBLCD::printf("T: %3.2f", targetBlockLocation);
		_delay_ms(1500);

		while (1) {
			uint16_t movedDistance = moveRobot(DISTANCE_TO_MOVE);
			moveParticles(movedDistance);
			redistributeParticles();
			StatsResult result = calculateVariance();

			if (result.variance < 15.0 && fabsf(result.mean - targetBlockLocation) < 3.0) {
				BBLCD::clear();
				BBLCD::set_cursor(0, 0);
				BBLCD::printf("TARGET");
				BBLCD::set_cursor(0, 1);
				BBLCD::printf("LOCATED");

				leftMotor.set_speed(-100);
				rightMotor.set_speed(-100);
				_delay_ms(700);
				leftMotor.set_speed(100);
				rightMotor.set_speed(-100);
				_delay_ms(2000);
				leftMotor.set_speed(0);
				rightMotor.set_speed(0);

				break;
			}

			BBLCD::clear();
			BBLCD::set_cursor(0, 0);
			BBLCD::printf("M%6.2f", result.mean);
			BBLCD::set_cursor(0, 1);
			BBLCD::printf("V%6.2f", result.variance);
		}
		while (1) {
			if (BBDigital::get_btn_press()) {
				break;
			}
		}
	}
}

