################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f765vgtx.s 

OBJS += \
./Core/Startup/startup_stm32f765vgtx.o 

S_DEPS += \
./Core/Startup/startup_stm32f765vgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Common" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/LED" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/WDog" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sche" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/UART" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/SysTick" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/I2C" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/Auto_run" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/DateTime" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/IR_LED" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/MB85RS2MT" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature/LT8722" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature/Heater" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature/NTC" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/LSM6DSOX" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/BME280" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/BMP390" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/H250" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/H3LIS331DL" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/K33" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto/min" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto/min_app" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/TEST_LaserPhoto/ADG1414" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/TEST_LaserPhoto/ADS8327" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/TEST_LaserPhoto/MCP4902" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/sst0_c/ports/arm-cm" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/container" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/Utils/ports/stm32f4xx" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/Utils" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/sst0_c/src" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/FSM" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/STDIO" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/Signals" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/shell/CLI_Command" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/app_main" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/shell" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/Board" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/CLI_Terminal" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MiddleWare/CLI_Terminal/CLI_Src" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/temperature_monitor" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/App/temperature_control" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/bsp_tec" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/bsp_ntc" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/BSP/bsp_heater" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f765vgtx.d ./Core/Startup/startup_stm32f765vgtx.o

.PHONY: clean-Core-2f-Startup

