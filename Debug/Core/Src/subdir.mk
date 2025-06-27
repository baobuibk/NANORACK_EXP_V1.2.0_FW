################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32F765xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Common" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Devices" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/LED" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Sensor_I2C" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/BSupport/UART" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/BSupport/SysTick" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/BSupport/I2C" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Devices/ADG1414" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Devices/DateTime" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Devices/IR_LED" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Devices/MB85RS2MT" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Sensor_I2C/LSM6DSOX" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Sensor_I2C/BME280" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Sensor_I2C/BMP390" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Sensor_I2C/H250" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Sensor_I2C/H3LIS331DL" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Sensor_I2C/K33" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MIN_Proto/min" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MIN_Proto/min_app" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MIN_Proto" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MiddleWare/sst0_c/ports/arm-cm" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MiddleWare/container" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MiddleWare/Utils/ports/stm32f4xx" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MiddleWare/Utils" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MiddleWare/sst0_c/src" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MiddleWare/FSM" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MiddleWare/STDIO" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/App/Signals" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/App/shell/CLI_Command" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/App/app_main" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/App/shell" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/BSupport/BSP/Board" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MiddleWare/CLI_Terminal" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/MiddleWare/CLI_Terminal/CLI_Src" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/App/temperature_monitor" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/App/temperature_control" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/BSupport/BSP/bsp_tec" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/BSupport/BSP/bsp_ntc" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/BSupport/BSP/bsp_heater" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Devices/ADS8327" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Devices/MCP4902" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/BSupport/BSP/bsp_photodiode" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Devices/IS66WVS4M8BLL" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Devices/LT8722" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/Devices/NTC" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/BSupport/BSP/bsp_laser" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/App/experiment" -I"D:/WorkSpace/STM32 PRJ/EXP_Firmware/EXP_DMA_V120/EXP_V120/Dev/BSupport/BSP/bsp_spi_ram" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su

.PHONY: clean-Core-2f-Src

