################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Audio/ConvertedMIDIFiles/MIDIFile_Boogie.c \
../src/Audio/ConvertedMIDIFiles/MIDIFile_Starwars.c \
../src/Audio/ConvertedMIDIFiles/MIDIFile_blastoff.c \
../src/Audio/ConvertedMIDIFiles/MIDIFile_gong.c 

OBJS += \
./src/Audio/ConvertedMIDIFiles/MIDIFile_Boogie.o \
./src/Audio/ConvertedMIDIFiles/MIDIFile_Starwars.o \
./src/Audio/ConvertedMIDIFiles/MIDIFile_blastoff.o \
./src/Audio/ConvertedMIDIFiles/MIDIFile_gong.o 

C_DEPS += \
./src/Audio/ConvertedMIDIFiles/MIDIFile_Boogie.d \
./src/Audio/ConvertedMIDIFiles/MIDIFile_Starwars.d \
./src/Audio/ConvertedMIDIFiles/MIDIFile_blastoff.d \
./src/Audio/ConvertedMIDIFiles/MIDIFile_gong.d 


# Each subdirectory must supply rules for building sources it contributes
src/Audio/ConvertedMIDIFiles/%.o: ../src/Audio/ConvertedMIDIFiles/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DRONE_V12 -DGCC_ARMCM3 -Dsourcerygxx -DPART_LM3S8962 -I"C:\Users\Sam\Documents\rone\software\system\driverlib" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS" -I"C:\Users\Sam\Documents\rone\software\system\roneos\inc" -I"C:\Users\Sam\Documents\rone\software\system\FreeRTOS\inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


