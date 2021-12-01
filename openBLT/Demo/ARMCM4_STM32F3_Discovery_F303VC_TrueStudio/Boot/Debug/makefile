################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include lib/STM32_USB_Device_Library/Core/Src/subdir.mk
-include lib/STM32F3xx_HAL_Driver/Src/subdir.mk
-include lib/subdir.mk
-include core/ARMCM4_STM32F3/GCC/subdir.mk
-include core/ARMCM4_STM32F3/subdir.mk
-include core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
DashBoard_Bootloader.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
DashBoard_Bootloader.list \

OBJCOPY_SREC += \
DashBoard_Bootloader.srec \


# All Target
all: DashBoard_Bootloader.elf secondary-outputs

# Tool invocations
DashBoard_Bootloader.elf: $(OBJS) $(USER_OBJS) /Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Demo/ARMCM4_STM32F3_Discovery_F303VC_TrueStudio/Boot/stm32f30_flash.ld
	arm-none-eabi-gcc -o "DashBoard_Bootloader.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Demo/ARMCM4_STM32F3_Discovery_F303VC_TrueStudio/Boot/stm32f30_flash.ld" --specs=nosys.specs -Wl,-Map="DashBoard_Bootloader.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

DashBoard_Bootloader.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "DashBoard_Bootloader.list"
	@echo 'Finished building: $@'
	@echo ' '

DashBoard_Bootloader.srec: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O srec $(EXECUTABLES) "DashBoard_Bootloader.srec"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_SREC)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
