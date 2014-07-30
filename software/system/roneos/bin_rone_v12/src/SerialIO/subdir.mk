################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/SerialIO/basicPrinting.c \
../src/SerialIO/cprintf.c \
../src/SerialIO/rprintf.c \
../src/SerialIO/serial.c \
../src/SerialIO/serialCommand.c \
../src/SerialIO/snprintf.c \
../src/SerialIO/systemCommands.c 

OBJS += \
./src/SerialIO/basicPrinting.o \
./src/SerialIO/cprintf.o \
./src/SerialIO/rprintf.o \
./src/SerialIO/serial.o \
./src/SerialIO/serialCommand.o \
./src/SerialIO/snprintf.o \
./src/SerialIO/systemCommands.o 

C_DEPS += \
./src/SerialIO/basicPrinting.d \
./src/SerialIO/cprintf.d \
./src/SerialIO/rprintf.d \
./src/SerialIO/serial.d \
./src/SerialIO/serialCommand.d \
./src/SerialIO/snprintf.d \
./src/SerialIO/systemCommands.d 


# Each subdirectory must supply rules for building sources it contributes
src/SerialIO/%.o: ../src/SerialIO/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DRONE_V12 -DGCC_ARMCM3 -Dsourcerygxx -DPART_LM3S8962 -I"C:\Users\Sam\Documents\rone\software\system\driverlib" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS" -I"C:\Users\Sam\Documents\rone\software\system\roneos\inc" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS\inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


