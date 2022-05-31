################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BLT/asserts.c \
../Core/Src/BLT/backdoor.c \
../Core/Src/BLT/boot.c \
../Core/Src/BLT/can.c \
../Core/Src/BLT/com.c \
../Core/Src/BLT/cop.c \
../Core/Src/BLT/cpu.c \
../Core/Src/BLT/flash.c \
../Core/Src/BLT/hooks.c \
../Core/Src/BLT/led.c \
../Core/Src/BLT/net.c \
../Core/Src/BLT/nvm.c \
../Core/Src/BLT/rs232.c \
../Core/Src/BLT/timer.c \
../Core/Src/BLT/xcp.c 

OBJS += \
./Core/Src/BLT/asserts.o \
./Core/Src/BLT/backdoor.o \
./Core/Src/BLT/boot.o \
./Core/Src/BLT/can.o \
./Core/Src/BLT/com.o \
./Core/Src/BLT/cop.o \
./Core/Src/BLT/cpu.o \
./Core/Src/BLT/flash.o \
./Core/Src/BLT/hooks.o \
./Core/Src/BLT/led.o \
./Core/Src/BLT/net.o \
./Core/Src/BLT/nvm.o \
./Core/Src/BLT/rs232.o \
./Core/Src/BLT/timer.o \
./Core/Src/BLT/xcp.o 

C_DEPS += \
./Core/Src/BLT/asserts.d \
./Core/Src/BLT/backdoor.d \
./Core/Src/BLT/boot.d \
./Core/Src/BLT/can.d \
./Core/Src/BLT/com.d \
./Core/Src/BLT/cop.d \
./Core/Src/BLT/cpu.d \
./Core/Src/BLT/flash.d \
./Core/Src/BLT/hooks.d \
./Core/Src/BLT/led.d \
./Core/Src/BLT/net.d \
./Core/Src/BLT/nvm.d \
./Core/Src/BLT/rs232.d \
./Core/Src/BLT/timer.d \
./Core/Src/BLT/xcp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/BLT/%.o Core/Src/BLT/%.su: ../Core/Src/BLT/%.c Core/Src/BLT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-BLT

clean-Core-2f-Src-2f-BLT:
	-$(RM) ./Core/Src/BLT/asserts.d ./Core/Src/BLT/asserts.o ./Core/Src/BLT/asserts.su ./Core/Src/BLT/backdoor.d ./Core/Src/BLT/backdoor.o ./Core/Src/BLT/backdoor.su ./Core/Src/BLT/boot.d ./Core/Src/BLT/boot.o ./Core/Src/BLT/boot.su ./Core/Src/BLT/can.d ./Core/Src/BLT/can.o ./Core/Src/BLT/can.su ./Core/Src/BLT/com.d ./Core/Src/BLT/com.o ./Core/Src/BLT/com.su ./Core/Src/BLT/cop.d ./Core/Src/BLT/cop.o ./Core/Src/BLT/cop.su ./Core/Src/BLT/cpu.d ./Core/Src/BLT/cpu.o ./Core/Src/BLT/cpu.su ./Core/Src/BLT/flash.d ./Core/Src/BLT/flash.o ./Core/Src/BLT/flash.su ./Core/Src/BLT/hooks.d ./Core/Src/BLT/hooks.o ./Core/Src/BLT/hooks.su ./Core/Src/BLT/led.d ./Core/Src/BLT/led.o ./Core/Src/BLT/led.su ./Core/Src/BLT/net.d ./Core/Src/BLT/net.o ./Core/Src/BLT/net.su ./Core/Src/BLT/nvm.d ./Core/Src/BLT/nvm.o ./Core/Src/BLT/nvm.su ./Core/Src/BLT/rs232.d ./Core/Src/BLT/rs232.o ./Core/Src/BLT/rs232.su ./Core/Src/BLT/timer.d ./Core/Src/BLT/timer.o ./Core/Src/BLT/timer.su ./Core/Src/BLT/xcp.d ./Core/Src/BLT/xcp.o ./Core/Src/BLT/xcp.su

.PHONY: clean-Core-2f-Src-2f-BLT

