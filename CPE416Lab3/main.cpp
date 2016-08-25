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

#include <bumblebee/globals.hpp>

#include "neural/NeuralNetwork.h"

const int leftSensorPin = 0;
const int rightSensorPin = 1;

const float leftZero = 0.14;
const float rightZero = 0.02;
const float leftMult = 1.8;
const float rightMult = 1.0;

const float leftMotorGain = 1.0;
const float rightMotorGain = 1.0;

const float baseSpeed = .15;
const float controllerP = .75;

const uint16_t MAX_MEASUREMENTS = 1024;

inline float readLight(uint8_t adcInput, float zeroPoint, float multiplier) {
	float adcReading = BBADC::read(adcInput) * 0.001953125;
	if (adcReading > zeroPoint) {
		float output = (1 - adcReading - zeroPoint) * multiplier;
		if (output > 1) {
			return 1;
		}
		return output;
	} else {
		return 0;
	}
}

typedef struct ControllerOutput {
	float leftSpeed;
	float rightSpeed;
} controllerOutput;

typedef struct LightMeasurement {
	float leftLight;
	float rightLight;
} LightMeasurement;

ControllerOutput proportionalController(LightMeasurement input) {
	ControllerOutput output;

	float error = input.leftLight - input.rightLight;
	float correction = error * controllerP;

	output.leftSpeed = leftMotorGain * (baseSpeed - correction);
	output.rightSpeed = rightMotorGain * (baseSpeed + correction);

	return output;
}

enum class RunMode
	: uint8_t {
		proportional, data, select_training_iterations, training, driving, driving_and_training
};

int main() {
	BBUtility::init();
	BBServo leftMotor(1);
	BBServo rightMotor(0);
	ControllerOutput controllerOutput;
	uint16_t dataCounter = 0;
	uint16_t trainingCount = 0;
	LightMeasurement measurements[MAX_MEASUREMENTS];

	srand(BBADC::read(leftSensorPin));
	int layers[2] = { 3, 2 };
	NeuralNetwork NN(2, layers, 2);

	RunMode mode = RunMode::data;
	BBLCD::set_cursor(0, 0);
//	BBLCD::printf("Proportional");

	while (1) {
		LightMeasurement lightSensors;
		lightSensors.leftLight = readLight(leftSensorPin, leftZero, leftMult);
		lightSensors.rightLight = readLight(rightSensorPin, rightZero, rightMult);

		if (mode == RunMode::proportional) {
			controllerOutput = proportionalController(lightSensors);

			BBLCD::set_cursor(0, 0);
			BBLCD::printf("L:%.2f", lightSensors.leftLight);
			BBLCD::set_cursor(0, 1);
			BBLCD::printf("R:%.2f", lightSensors.rightLight);

			if (BBDigital::get_btn_press()) {
				mode = RunMode::data;
				dataCounter = 0;
				BBLCD::clear();
				_delay_ms(100);
			}

		} else if (mode == RunMode::data) {
			controllerOutput.leftSpeed = 0;
			controllerOutput.rightSpeed = 0;

			measurements[dataCounter].leftLight = lightSensors.leftLight;
			measurements[dataCounter].rightLight = lightSensors.rightLight;

			BBLCD::set_cursor(0, 0);
			BBLCD::printf("Data %4d", dataCounter);
			BBLCD::set_cursor(0, 1);
			BBLCD::printf("%.1f %.1f", lightSensors.leftLight, lightSensors.rightLight);

			++dataCounter;

			if (BBDigital::get_btn_press()) {
				mode = RunMode::select_training_iterations;
				BBLCD::clear();
				 _delay_ms(100);
			}

		} else if (mode == RunMode::select_training_iterations) {

			controllerOutput.leftSpeed = 0;
			controllerOutput.rightSpeed = 0;

			trainingCount = BBAccelerometer::get_x() * 10;
			BBLCD::set_cursor(0, 0);
			BBLCD::printf("Count:");
			BBLCD::set_cursor(0, 1);
			BBLCD::printf("%0d", trainingCount);

			if (BBDigital::get_btn_press()) {
				mode = RunMode::training;
				BBLCD::clear();
				_delay_ms(100);
			}


		} else if (mode == RunMode::training) {
			controllerOutput.leftSpeed = 0;
			controllerOutput.rightSpeed = 0;
			BBLCD::set_cursor(0, 0);
			BBLCD::printf("Training");

			uint16_t dataIndex = 0;

			while (--trainingCount) {
				if (dataIndex == dataCounter) {
					dataIndex = 0;
				}

				float *input = (float *) (measurements + dataIndex);
				ControllerOutput desiredOutput = proportionalController(measurements[dataIndex]);
				NN.compute(input);
				NN.train(input, (float *) &desiredOutput);

				++dataIndex;
			}

			if (BBDigital::get_btn_press()) {
				mode = RunMode::driving;
				BBLCD::clear();
				_delay_ms(100);
				BBLCD::printf("Driving");
			}

		} else if (mode == RunMode::driving) {

			float *output;
			output = NN.compute((float *) &lightSensors);

			controllerOutput.leftSpeed = output[0];
			controllerOutput.rightSpeed = output[1];

			if (BBDigital::get_btn_press()) {
				mode = RunMode::select_training_iterations;
				BBLCD::clear();
				_delay_ms(100);
			}
		}

		leftMotor.set_speed(100 * controllerOutput.leftSpeed);
		rightMotor.set_speed(-100 * controllerOutput.rightSpeed);

	}
}

