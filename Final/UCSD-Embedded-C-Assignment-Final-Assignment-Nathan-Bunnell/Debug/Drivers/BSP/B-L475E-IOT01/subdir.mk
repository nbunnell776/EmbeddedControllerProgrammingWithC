################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.c \
../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.c 

OBJS += \
./Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.o \
./Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.o 

C_DEPS += \
./Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.d \
./Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.o: ../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.c Drivers/BSP/B-L475E-IOT01/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-Final-Assignment-Nathan-Bunnell/Drivers/BSP/B-L475E-IOT01" -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-Final-Assignment-Nathan-Bunnell/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.o: ../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.c Drivers/BSP/B-L475E-IOT01/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-Final-Assignment-Nathan-Bunnell/Drivers/BSP/B-L475E-IOT01" -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-Final-Assignment-Nathan-Bunnell/Drivers/BSP/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

