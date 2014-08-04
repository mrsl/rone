################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := cs-rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/System/subdir.mk
-include src/SerialIO/subdir.mk
-include src/Sensors/subdir.mk
-include src/Motors/subdir.mk
-include src/InputOutput/Logger/Option/subdir.mk
-include src/InputOutput/Logger/subdir.mk
-include src/InputOutput/subdir.mk
-include src/IRComms/subdir.mk
-include src/Audio/ConvertedMIDIFiles/subdir.mk
-include src/Audio/subdir.mk
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

# All Target
all: libroneos.a

# Tool invocations
libroneos.a: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM GNU Archiver'
	arm-none-eabi-ar -r  "libroneos.a" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(ARCHIVES)$(S_UPPER_DEPS) libroneos.a
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
