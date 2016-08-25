/*
 * BBAccelerometer.hpp
 *
 *  Created on: Sep 26, 2015
 *      Author: thomaswillson
 */

#pragma once

#include "globals.hpp"

class BBAccelerometer {
public:
   inline static u08 get_x() {
      return BBI2C::read_register(MMA8453_ADDR, 0x1);
   }

   inline static u08 get_y() {
      return BBI2C::read_register(MMA8453_ADDR, 0x3);
   }

   inline static u08 get_z() {
      return BBI2C::read_register(MMA8453_ADDR, 0x5);
   }
private:
   static const u08 MMA8453_ADDR = 0x1C;
};

