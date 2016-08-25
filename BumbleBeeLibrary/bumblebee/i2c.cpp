#include <util/delay.h>
#include <avr/io.h>

#include "globals.hpp"
#include "i2c.hpp"

#define SDA_PIN 5
#define SCL_PIN 4

#define I2C_DDR DDRE
#define I2C_PORT PORTE
#define I2C_PIN PINE

#define SDA_HI I2C_DDR &= ~_BV(SDA_PIN); //make as input to set high
#define SCL_HI I2C_DDR &= ~_BV(SCL_PIN); //make as input to set high
#define SDA_LO I2C_DDR |= _BV(SDA_PIN); I2C_PORT &= ~_BV(SDA_PIN); //set as output and write low
#define SCL_LO I2C_DDR |= _BV(SCL_PIN); I2C_PORT &= ~_BV(SCL_PIN); //set as output and write low

#define I2C_DELAY 1

const u08 COMPASS_ADDR = 0x1E;

void BBI2C::start() {
   //set both to high
   I2C_DDR &= ~(_BV(SDA_PIN) | _BV(SCL_PIN)); //make as input to set high
   _delay_us(I2C_DELAY);

   SDA_LO
   ;
   _delay_us(I2C_DELAY);

   SCL_LO
   ;
   _delay_us(I2C_DELAY);
}

void BBI2C::stop() {
   SCL_HI
   ;
   _delay_us(I2C_DELAY);

   SDA_HI
   ;
   _delay_us(I2C_DELAY);
}

void BBI2C::unlock() {
   SDA_HI
   ;
   _delay_us(I2C_DELAY);

   while (!(I2C_PIN & _BV(SDA_PIN))) {
      SCL_HI
      ;
      _delay_us(I2C_DELAY);
      SCL_LO
      ;
      _delay_us(I2C_DELAY);
   }

   SCL_HI
   ;
}

void BBI2C::clock_scl() {
   SCL_HI
   ;
   _delay_us(I2C_DELAY);
   SCL_LO
   ;
   _delay_us(I2C_DELAY);
}

//send the register address to read
void BBI2C::send_address(u08 addr, u08 reg, u08 read) {
   unlock();
   start(); //send start bit

   //send address bits 6:0
   for (u08 i = 0; i < 7; i++) {
      if (addr & _BV(6)) { //send 1
         SDA_HI
         ;
      } else { //send 0
         SDA_LO
         ;
      }

      _delay_us(I2C_DELAY);
      clock_scl();

      addr = addr << 1;
   }

   //send W bit of 0
   SDA_LO
   ;
   _delay_us(I2C_DELAY);
   clock_scl();

   SDA_HI
   ; //release the data line
   _delay_us(1);
   SCL_HI
   ;

   //read the ACK
   if (I2C_PIN & _BV(SDA_PIN)) {
      BBLCD::print("addrNACK");
      while (1) {
      }
   } else {
      //received ACK
      //print_string ("ACK",3);
   }
   _delay_us(I2C_DELAY);
   SCL_LO
   ;
   _delay_us(I2C_DELAY);

   /////////send the register address
   //register address
   for (u08 i = 0; i < 8; i++) {
      if (reg & _BV(7)) { //send 1
         SDA_HI
         ;
      } else { //send 0
         SDA_LO
         ;
      }

      _delay_us(I2C_DELAY);
      clock_scl();

      reg = reg << 1;
   }

   SDA_HI
   ; //release the data line
   _delay_us(1);
   SCL_HI
   ;

   //read the ACK
   if (I2C_PIN & _BV(SDA_PIN)) {
      BBLCD::print("NACK");
      while (1) {
      }
   } else {
      //received ACK
      //print_string ("ACK",3);
   }
   _delay_us(I2C_DELAY);
   SCL_LO
   ;
   _delay_us(I2C_DELAY);

   if (read)
      stop();
}

/* */
void BBI2C::write_register_data(u08* data, u08 num) {
   u08 i, j;

   for (j = 0; j < num; j++) {
      //send data bits 7:0
      for (i = 0; i < 8; i++) {
         if (*data & _BV(7)) { //send 1
            SDA_HI
            ;
         } else { //send 0
            SDA_LO
            ;
         }

         _delay_us(I2C_DELAY);
         clock_scl();

         *data = *data << 1;
      }

      data++;

      SDA_HI
      ; //release the data line
      _delay_us(1);
      SCL_HI
      ;

      //read the ACK
      if (I2C_PIN & _BV(SDA_PIN)) {
         BBLCD::print("wr_NACK");
         while (1) {
         }
      } else {
      }

      _delay_us(I2C_DELAY);
      SCL_LO
      ;
      _delay_us(I2C_DELAY);
   }

   stop();

}

void BBI2C::read_register_data(u08 addr, u08* data, u08 num) {
   u08 i, j;
   //u08 addr = MMA8453_ADDR;
   //addr = COMPASS_ADDR; //compass

   start(); //send start bit

   //send device address bits 6:0
   for (i = 0; i < 7; i++) {
      if (addr & _BV(6)) { //send 1
         SDA_HI
         ;
      } else { //send 0
         SDA_LO
         ;
      }

      _delay_us(I2C_DELAY);
      clock_scl();

      addr = addr << 1;
   }

   //send R bit of 1
   SDA_HI
   ;
   _delay_us(I2C_DELAY);
   clock_scl();

   SDA_HI
   ; //release the data line
   _delay_us(1);
   SCL_HI
   ;

   //read the ACK
   if (I2C_PIN & _BV(SDA_PIN)) {
      BBLCD::print("readNACK");
      while (1) {
      }
   } else {
      //received ACK
      //print_string ("ACK",3);
   }
   _delay_us(I2C_DELAY);
   SCL_LO
   ;
   _delay_us(I2C_DELAY);

   /////////read the data from the device
   SDA_HI
   ; //release the data line
   _delay_us(1);
   for (j = 0; j < num; j++) {
      for (i = 0; i < 8; i++) {
         *data = *data << 1;
         SCL_HI
         ;
         //_delay_us(I2C_DELAY/2);
         _delay_us(1);

         if (I2C_PIN & _BV(SDA_PIN)) {
            *data |= 1;
         }

         //_delay_us(I2C_DELAY/2);
         _delay_us(1);
         SCL_LO
         ;

         _delay_us(I2C_DELAY);
      }

      //master sends ACK
      SDA_LO
      ;
      _delay_us(1);
      clock_scl();

      data++; //move pointer to the next byte
   }

   _delay_us(1);

   stop();
}

void BBI2C::write_register(u08 dev_addr, u08 address, u08 data) {
   send_address(dev_addr, address, 0);
   write_register_data(&data, 1);
}

u08 BBI2C::read_register(u08 dev_addr, u08 address) {
   u08 temp;
   send_address(dev_addr, address, 1);
   read_register_data(dev_addr, &temp, 1);
   return temp;
}
