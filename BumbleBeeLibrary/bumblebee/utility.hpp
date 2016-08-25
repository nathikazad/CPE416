/*
 * utility.hpp
 *
 *  Created on: Sep 26, 2015
 *      Author: thomaswillson
 */

#pragma once

#include <util/delay.h>

class BBUtility {
public:
   static void init(void);

   inline static void delay_ms(int ms) {
      for (int i = 0; i < ms; ++i) {
         _delay_us(1000);
      }
   }

};
