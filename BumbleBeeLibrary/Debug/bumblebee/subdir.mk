################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../bumblebee/acclerometer.cpp \
../bumblebee/adc.cpp \
../bumblebee/digital.cpp \
../bumblebee/i2c.cpp \
../bumblebee/lcd.cpp \
../bumblebee/motor.cpp \
../bumblebee/servo.cpp \
../bumblebee/utility.cpp 

OBJS += \
./bumblebee/acclerometer.o \
./bumblebee/adc.o \
./bumblebee/digital.o \
./bumblebee/i2c.o \
./bumblebee/lcd.o \
./bumblebee/motor.o \
./bumblebee/servo.o \
./bumblebee/utility.o 

CPP_DEPS += \
./bumblebee/acclerometer.d \
./bumblebee/adc.d \
./bumblebee/digital.d \
./bumblebee/i2c.d \
./bumblebee/lcd.d \
./bumblebee/motor.d \
./bumblebee/servo.d \
./bumblebee/utility.d 


# Each subdirectory must supply rules for building sources it contributes
bumblebee/%.o: ../bumblebee/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -std=gnu++11 -mmcu=atmega645 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


