################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/InputOutput/blinkyLed.c \
../src/InputOutput/buttons.c \
../src/InputOutput/ir_beacon.c \
../src/InputOutput/leds.c \
../src/InputOutput/radio.c \
../src/InputOutput/radioCommand.c 

OBJS += \
./src/InputOutput/blinkyLed.o \
./src/InputOutput/buttons.o \
./src/InputOutput/ir_beacon.o \
./src/InputOutput/leds.o \
./src/InputOutput/radio.o \
./src/InputOutput/radioCommand.o 

C_DEPS += \
./src/InputOutput/blinkyLed.d \
./src/InputOutput/buttons.d \
./src/InputOutput/ir_beacon.d \
./src/InputOutput/leds.d \
./src/InputOutput/radio.d \
./src/InputOutput/radioCommand.d 


# Each subdirectory must supply rules for building sources it contributes
src/InputOutput/%.o: ../src/InputOutput/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DRONE_V12 -DGCC_ARMCM3 -Dsourcerygxx -DPART_LM3S8962 -I"C:\Users\Sam\Documents\rone\software\system\driverlib" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS" -I"C:\Users\Sam\Documents\rone\software\system\roneos\inc" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS\inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


