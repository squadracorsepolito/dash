################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Source/ARMCM4_STM32F3/GCC/cpu_comp.c 

OBJS += \
./core/ARMCM4_STM32F3/GCC/cpu_comp.o 

C_DEPS += \
./core/ARMCM4_STM32F3/GCC/cpu_comp.d 


# Each subdirectory must supply rules for building sources it contributes
core/ARMCM4_STM32F3/GCC/cpu_comp.o: /Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Source/ARMCM4_STM32F3/GCC/cpu_comp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F303xC -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -c -I/Users/bigfede/Documents/Repository/squadraCorse/SC19/TLBoard/Software/openBLT/Source -I/Users/bigfede/Documents/Repository/squadraCorse/SC19/TLBoard/Software/openBLT/Source/ARMCM4_STM32F3 -I"/Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Demo/ARMCM4_STM32F3_Discovery_F303VC_TrueStudio/Boot" -I"/Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Demo/ARMCM4_STM32F3_Discovery_F303VC_TrueStudio/Boot/lib" -I"/Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Demo/ARMCM4_STM32F3_Discovery_F303VC_TrueStudio/Boot/lib/CMSIS/Include" -I"/Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Demo/ARMCM4_STM32F3_Discovery_F303VC_TrueStudio/Boot/lib/CMSIS/Device/ST/STM32F3xx/Include" -I"/Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Demo/ARMCM4_STM32F3_Discovery_F303VC_TrueStudio/Boot/lib/STM32F3xx_HAL_Driver/Inc" -I"/Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Demo/ARMCM4_STM32F3_Discovery_F303VC_TrueStudio/Boot/lib/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/Users/bigfede/Documents/Repository/squadraCorse/SC19/DashBoard/Software/openBLT/Demo/ARMCM4_STM32F3_Discovery_F303VC_TrueStudio/Boot/lib/STM32_USB_Device_Library/Core/Inc" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"core/ARMCM4_STM32F3/GCC/cpu_comp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

