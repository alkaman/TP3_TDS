################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/planificador/miniplanificador.c 

OBJS += \
./Core/planificador/miniplanificador.o 

C_DEPS += \
./Core/planificador/miniplanificador.d 


# Each subdirectory must supply rules for building sources it contributes
Core/planificador/miniplanificador.o: ../Core/planificador/miniplanificador.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/planificador/miniplanificador.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

