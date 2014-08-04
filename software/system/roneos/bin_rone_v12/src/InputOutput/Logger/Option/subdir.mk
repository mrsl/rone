################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/InputOutput/Logger/Option/ccsbcs.c 

OBJS += \
./src/InputOutput/Logger/Option/ccsbcs.o 

C_DEPS += \
./src/InputOutput/Logger/Option/ccsbcs.d 


# Each subdirectory must supply rules for building sources it contributes
src/InputOutput/Logger/Option/%.o: ../src/InputOutput/Logger/Option/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DRONE_V12 -DGCC_ARMCM3 -Dsourcerygxx -DPART_LM3S8962 -I"C:\Users\Sam\Documents\rone\software\system\driverlib" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS" -I"C:\Users\Sam\Documents\rone\software\system\roneos\inc" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS\inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


