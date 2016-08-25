/*
 * adc.hpp
 *
 *  Created on: Sep 26, 2015
 *      Author: thomaswillson
 */

#pragma once

#include <avr/io.h>

#include "globals.hpp"

class BBADC {
public:
   inline static u08 read(u08 num) {
      ADMUX &= 0xF8;
      ADMUX |= num;

      ADCSRA |= _BV(ADSC);

      while (ADCSRA & _BV(ADSC)) {
      }

      return ADCH;
   }

   static void init();
};
