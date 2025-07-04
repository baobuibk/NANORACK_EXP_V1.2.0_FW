################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Dev/BSupport/BSP/Board/board.c \
../Dev/BSupport/BSP/Board/bsp_board_EXP_1_2_0.c \
../Dev/BSupport/BSP/Board/bsp_debug.c 

OBJS += \
./Dev/BSupport/BSP/Board/board.o \
./Dev/BSupport/BSP/Board/bsp_board_EXP_1_2_0.o \
./Dev/BSupport/BSP/Board/bsp_debug.o 

C_DEPS += \
./Dev/BSupport/BSP/Board/board.d \
./Dev/BSupport/BSP/Board/bsp_board_EXP_1_2_0.d \
./Dev/BSupport/BSP/Board/bsp_debug.d 


# Each subdirectory must supply rules for building sources it contributes
Dev/BSupport/BSP/Board/%.o Dev/BSupport/BSP/Board/%.su Dev/BSupport/BSP/Board/%.cyclo: ../Dev/BSupport/BSP/Board/%.c Dev/BSupport/BSP/Board/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32F765xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Common" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/LED" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/UART" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/SysTick" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/I2C" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/ADG1414" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/DateTime" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/IR_LED" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/MB85RS2MT" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/LSM6DSOX" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/BME280" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/BMP390" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/H250" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/H3LIS331DL" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/K33" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto/min" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto/min_app" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/sst0_c/ports/arm-cm" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/container" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/Utils/ports/stm32f4xx" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/Utils" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/sst0_c/src" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/FSM" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/STDIO" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/Signals" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/shell/CLI_Command" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/app_main" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/shell" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/Board" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/CLI_Terminal" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/CLI_Terminal/CLI_Src" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/temperature_monitor" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/temperature_control" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/bsp_tec" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/bsp_ntc" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/bsp_heater" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/ADS8327" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/MCP4902" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/bsp_photodiode" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/IS66WVS4M8BLL" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/LT8722" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/NTC" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/bsp_laser" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/experiment" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/bsp_spi_ram" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Dev-2f-BSupport-2f-BSP-2f-Board

clean-Dev-2f-BSupport-2f-BSP-2f-Board:
	-$(RM) ./Dev/BSupport/BSP/Board/board.cyclo ./Dev/BSupport/BSP/Board/board.d ./Dev/BSupport/BSP/Board/board.o ./Dev/BSupport/BSP/Board/board.su ./Dev/BSupport/BSP/Board/bsp_board_EXP_1_2_0.cyclo ./Dev/BSupport/BSP/Board/bsp_board_EXP_1_2_0.d ./Dev/BSupport/BSP/Board/bsp_board_EXP_1_2_0.o ./Dev/BSupport/BSP/Board/bsp_board_EXP_1_2_0.su ./Dev/BSupport/BSP/Board/bsp_debug.cyclo ./Dev/BSupport/BSP/Board/bsp_debug.d ./Dev/BSupport/BSP/Board/bsp_debug.o ./Dev/BSupport/BSP/Board/bsp_debug.su

.PHONY: clean-Dev-2f-BSupport-2f-BSP-2f-Board

