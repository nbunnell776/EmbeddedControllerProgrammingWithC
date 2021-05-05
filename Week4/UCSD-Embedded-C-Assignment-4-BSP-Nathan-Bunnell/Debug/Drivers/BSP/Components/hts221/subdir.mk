################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/hts221/hts221.c \
../Drivers/BSP/Components/hts221/hts221_reg.c 

OBJS += \
./Drivers/BSP/Components/hts221/hts221.o \
./Drivers/BSP/Components/hts221/hts221_reg.o 

C_DEPS += \
./Drivers/BSP/Components/hts221/hts221.d \
./Drivers/BSP/Components/hts221/hts221_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/hts221/hts221.o: ../Drivers/BSP/Components/hts221/hts221.c Drivers/BSP/Components/hts221/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/hts221 -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/B-L475E-IOT01" -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/Components/Common" -I../Drivers/BSP/B-L475E-IOT01A1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/hts221/hts221.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/Components/hts221/hts221_reg.o: ../Drivers/BSP/Components/hts221/hts221_reg.c Drivers/BSP/Components/hts221/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/hts221 -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/B-L475E-IOT01" -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/Components/Common" -I../Drivers/BSP/B-L475E-IOT01A1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/hts221/hts221_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

