#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "globals.hpp"
#include "utility.hpp"

void BBUtility::init(void) {
   //make TX pin output
   DDRD |= _BV(2);

   //set i2c pins to outputs for testing
   //DDRE |= _BV(4) | _BV(5);

   BBADC::init();
   BBLCD::init();
#ifndef BOOTLOADER
   BBServo::init();
#endif
   BBMotor::init();
   BBDigital::init();

   //initialize the accelerometer
#ifndef BOOTLOADER
   BBI2C::write_register(0x1C, 0x2A, 0x1);  //change to WAKE mode
   _delay_ms(100);
   BBI2C::unlock();
#endif
}
