################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/frame_parser.c \
../Src/inputs.c \
../Src/lcd.c \
../Src/main.c \
../Src/rs485.c \
../Src/stm32f0xx_hal_msp.c \
../Src/stm32f0xx_it.c \
../Src/system_stm32f0xx.c 

OBJS += \
./Src/frame_parser.o \
./Src/inputs.o \
./Src/lcd.o \
./Src/main.o \
./Src/rs485.o \
./Src/stm32f0xx_hal_msp.o \
./Src/stm32f0xx_it.o \
./Src/system_stm32f0xx.o 

C_DEPS += \
./Src/frame_parser.d \
./Src/inputs.d \
./Src/lcd.d \
./Src/main.d \
./Src/rs485.d \
./Src/stm32f0xx_hal_msp.d \
./Src/stm32f0xx_it.d \
./Src/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -std=gnu11 -DUSE_FULL_LL_DRIVER -DSTM32F042x6 -DUSE_HAL_DRIVER '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -I"/home/proxima/Documents/git/BlueRovController/RovController/Inc" -I"/home/proxima/Documents/git/BlueRovController/RovController/Drivers/STM32F0xx_HAL_Driver/Inc" -I"/home/proxima/Documents/git/BlueRovController/RovController/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"/home/proxima/Documents/git/BlueRovController/RovController/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"/home/proxima/Documents/git/BlueRovController/RovController/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


