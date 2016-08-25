#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "globals.hpp"
#include "adc.hpp"

#define ANALOG0_PIN 0 //PF0
#define ANALOG1_PIN 1 //PF1
#define ANALOG2_PIN 2 //PF2
#define ANALOG3_PIN 3 //PF3
#define ANALOG4_PIN 4 //PF4
#define ANALOG5_PIN 5 //PF5
#define BATTERY_PIN 6 //PF6

void BBADC::init() {
   ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
   ADMUX |= _BV(ADLAR) | _BV(REFS0);
   return;
}

