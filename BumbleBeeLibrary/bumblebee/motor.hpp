/*
 * motor.hpp
 *
 *  Created on: Sep 26, 2015
 *      Author: thomaswillson
 */

#pragma once

class BBMotor {

public:
   static void init(void);
   static void test(void);

   BBMotor(u08 motorNumber);
   void set_speed(signed char speed);

private:
   u08 motorNumber;
};
