################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS/MemMang/heap_4.c 

OBJS += \
./FreeRTOS/MemMang/heap_4.o 

C_DEPS += \
./FreeRTOS/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/MemMang/heap_4.o: ../FreeRTOS/MemMang/heap_4.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/fat/FreeRTOS/include" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/fat/FreeRTOS/portable/ARM_CM4F" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"FreeRTOS/MemMang/heap_4.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

