################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := cs-rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
BilliardBots.bin \

SECONDARY_SIZE += \
BilliardBots.siz \


# All Target
all: BilliardBots.elf secondary-outputs

# Tool invocations
BilliardBots.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -T "C:\Users\Sam\Documents\rone\software\system\roneos\lm3s8962-rom.ld" -Xlinker --gc-sections -L"C:\Users\Sam\Documents\rone\software\system\roneLib\bin" -L"C:\Users\Sam\Documents\rone\software\system\roneos\bin_rone_v12" -L"C:\Users\Sam\Documents\rone\software\system\FreeRTOS\bin" -L"C:\Users\Sam\Documents\rone\software\system\driverlib\bin" -Wl,-Map,"BilliardBots.map" -o "BilliardBots.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

BilliardBots.bin: BilliardBots.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O binary "BilliardBots.elf"  "BilliardBots.bin"
	@echo 'Finished building: $@'
	@echo ' '

BilliardBots.siz: BilliardBots.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "BilliardBots.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(S_UPPER_DEPS) BilliardBots.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
