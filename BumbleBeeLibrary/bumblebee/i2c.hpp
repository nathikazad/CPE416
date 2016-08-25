#pragma once

#include "globals.hpp"

class BBI2C {

public:
   static void start();
   static void stop();
   static void unlock();
   static void clock_scl();

   static void write_register(u08 dev_addr, u08 address, u08 data);
   static u08 read_register(u08 dev_addr, u08 address);

private:
   static void send_address(u08 addr, u08 reg, u08 read);
   static void write_register_data(u08* data, u08 num);
   static void read_register_data(u08 addr, u08* data, u08 num);
};
