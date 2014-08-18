################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/cleanFlock.c 

OBJS += \
./src/cleanFlock.o 

C_DEPS += \
./src/cleanFlock.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -I"C:\Users\Sam\Documents\rone\software\system\driverlib\inc" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS\inc" -I"C:\Users\Sam\Documents\rone\software\system\roneos\inc" -I"C:\Users\Sam\Documents\rone\software\system\roneLib\inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


