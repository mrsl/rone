################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/irBilliardsTest\ .c 

OBJS += \
./src/irBilliardsTest\ .o 

C_DEPS += \
./src/irBilliardsTest\ .d 


# Each subdirectory must supply rules for building sources it contributes
src/irBilliardsTest\ .o: ../src/irBilliardsTest\ .c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -I"C:\Users\Sam\Documents\rone\software\system\driverlib\inc" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS\inc" -I"C:\Users\Sam\Documents\rone\software\system\roneos\inc" -I"C:\Users\Sam\Documents\rone\software\system\roneLib\inc" -std=gnu11 -MMD -MP -MF"src/irBilliardsTest .d" -MT"src/irBilliardsTest\ .d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


