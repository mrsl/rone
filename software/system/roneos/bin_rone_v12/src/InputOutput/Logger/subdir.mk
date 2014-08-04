################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/InputOutput/Logger/crc_ccitt.c \
../src/InputOutput/Logger/diskio.c \
../src/InputOutput/Logger/ff.c \
../src/InputOutput/Logger/logger.c \
../src/InputOutput/Logger/sd_card.c 

OBJS += \
./src/InputOutput/Logger/crc_ccitt.o \
./src/InputOutput/Logger/diskio.o \
./src/InputOutput/Logger/ff.o \
./src/InputOutput/Logger/logger.o \
./src/InputOutput/Logger/sd_card.o 

C_DEPS += \
./src/InputOutput/Logger/crc_ccitt.d \
./src/InputOutput/Logger/diskio.d \
./src/InputOutput/Logger/ff.d \
./src/InputOutput/Logger/logger.d \
./src/InputOutput/Logger/sd_card.d 


# Each subdirectory must supply rules for building sources it contributes
src/InputOutput/Logger/%.o: ../src/InputOutput/Logger/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DRONE_V12 -DGCC_ARMCM3 -Dsourcerygxx -DPART_LM3S8962 -I"C:\Users\Sam\Documents\rone\software\system\driverlib" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS" -I"C:\Users\Sam\Documents\rone\software\system\roneos\inc" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS\inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


