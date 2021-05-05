################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/main.o: ../Core/Src/main.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/hts221 -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/B-L475E-IOT01" -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/Components/Common" -I../Drivers/BSP/B-L475E-IOT01A1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32l4xx_hal_msp.o: ../Core/Src/stm32l4xx_hal_msp.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/hts221 -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/B-L475E-IOT01" -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/Components/Common" -I../Drivers/BSP/B-L475E-IOT01A1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32l4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32l4xx_it.o: ../Core/Src/stm32l4xx_it.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/hts221 -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/B-L475E-IOT01" -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/Components/Common" -I../Drivers/BSP/B-L475E-IOT01A1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32l4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/syscalls.o: ../Core/Src/syscalls.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/hts221 -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/B-L475E-IOT01" -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/Components/Common" -I../Drivers/BSP/B-L475E-IOT01A1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/sysmem.o: ../Core/Src/sysmem.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/hts221 -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/B-L475E-IOT01" -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/Components/Common" -I../Drivers/BSP/B-L475E-IOT01A1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/system_stm32l4xx.o: ../Core/Src/system_stm32l4xx.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/hts221 -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/B-L475E-IOT01" -I"C:/ST/STM32CubeIDE_1.6.1/STM32CubeIDE/Projects/UCSD-Embedded-C-Assignment-4-BSP-Nathan-Bunnell/Drivers/BSP/Components/Common" -I../Drivers/BSP/B-L475E-IOT01A1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/system_stm32l4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

