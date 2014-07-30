################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/System/charger.c \
../src/System/intMath.c \
../src/System/msp430Bootloader.c \
../src/System/pwm.c \
../src/System/robot_names.c \
../src/System/spi.c \
../src/System/spi_message.c \
../src/System/system.c \
../src/System/systemIO.c 

S_UPPER_SRCS += \
../src/System/lm3s8962-reset.S 

OBJS += \
./src/System/charger.o \
./src/System/intMath.o \
./src/System/lm3s8962-reset.o \
./src/System/msp430Bootloader.o \
./src/System/pwm.o \
./src/System/robot_names.o \
./src/System/spi.o \
./src/System/spi_message.o \
./src/System/system.o \
./src/System/systemIO.o 

C_DEPS += \
./src/System/charger.d \
./src/System/intMath.d \
./src/System/msp430Bootloader.d \
./src/System/pwm.d \
./src/System/robot_names.d \
./src/System/spi.d \
./src/System/spi_message.d \
./src/System/system.d \
./src/System/systemIO.d 

S_UPPER_DEPS += \
./src/System/lm3s8962-reset.d 


# Each subdirectory must supply rules for building sources it contributes
src/System/%.o: ../src/System/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DRONE_V12 -DGCC_ARMCM3 -Dsourcerygxx -DPART_LM3S8962 -I"C:\Users\Sam\Documents\rone\software\system\driverlib" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS" -I"C:\Users\Sam\Documents\rone\software\system\roneos\inc" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS\inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/System/%.o: ../src/System/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -x assembler-with-cpp -I"C:\Users\Sam\Documents\rone\software\system\driverlib" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


