################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Sensors/accelerometer.c \
../src/Sensors/bump_sensor.c \
../src/Sensors/gyro.c \
../src/Sensors/light_sensor.c 

OBJS += \
./src/Sensors/accelerometer.o \
./src/Sensors/bump_sensor.o \
./src/Sensors/gyro.o \
./src/Sensors/light_sensor.o 

C_DEPS += \
./src/Sensors/accelerometer.d \
./src/Sensors/bump_sensor.d \
./src/Sensors/gyro.d \
./src/Sensors/light_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
src/Sensors/%.o: ../src/Sensors/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DRONE_V12 -DGCC_ARMCM3 -Dsourcerygxx -DPART_LM3S8962 -I"C:\Users\Sam\Documents\rone\software\system\driverlib" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS" -I"C:\Users\Sam\Documents\rone\software\system\roneos\inc" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS\inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


