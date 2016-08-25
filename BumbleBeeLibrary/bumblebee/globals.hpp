#pragma once

#include<stdint.h>

typedef unsigned char u08;
typedef unsigned int u16;

#define sbi(a, b) (a) |= _BV(b)
#define cbi(a, b) (a) &= ~_BV(b)

#define USB_DETECT_PIN 2 //PG2
#define IR_PIN 0 //PD0
#define ACCEL_INT_PIN 1 //PD1

#include "adc.hpp"
#include "digital.hpp"
#include "utility.hpp"
#include "lcd.hpp"
#include "i2c.hpp"
#include "motor.hpp"
#include "servo.hpp"
#include "accelerometer.hpp"

