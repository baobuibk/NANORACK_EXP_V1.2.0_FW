################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Dev/Temperature/Heater/heater.c 

OBJS += \
./Dev/Temperature/Heater/heater.o 

C_DEPS += \
./Dev/Temperature/Heater/heater.d 


# Each subdirectory must supply rules for building sources it contributes
Dev/Temperature/Heater/%.o Dev/Temperature/Heater/%.su Dev/Temperature/Heater/%.cyclo: ../Dev/Temperature/Heater/%.c Dev/Temperature/Heater/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32F765xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I"D:/STworkspace/EXP_V120/Dev/Temperature" -I"D:/STworkspace/EXP_V120/Dev/WDog" -I"D:/STworkspace/EXP_V120/Dev/Board" -I"D:/STworkspace/EXP_V120/Dev/CLI_Terminal" -I"D:/STworkspace/EXP_V120/Dev/Common" -I"D:/STworkspace/EXP_V120/Dev/Devices" -I"D:/STworkspace/EXP_V120/Dev/LED" -I"D:/STworkspace/EXP_V120/Dev/Sensor_I2C" -I"D:/STworkspace/EXP_V120/Dev/Sche" -I"D:/STworkspace/EXP_V120/Dev/BSupport/UART" -I"D:/STworkspace/EXP_V120/Dev/BSupport/SysTick" -I"D:/STworkspace/EXP_V120/Dev/BSupport/I2C" -I"D:/STworkspace/EXP_V120/Dev/Devices/Auto_run" -I"D:/STworkspace/EXP_V120/Dev/Devices/DateTime" -I"D:/STworkspace/EXP_V120/Dev/Devices/IR_LED" -I"D:/STworkspace/EXP_V120/Dev/Devices/MB85RS2MT" -I"D:/STworkspace/EXP_V120/Dev/Temperature/LT8722" -I"D:/STworkspace/EXP_V120/Dev/Temperature/Heater" -I"D:/STworkspace/EXP_V120/Dev/Temperature/NTC" -I"D:/STworkspace/EXP_V120/Dev/Sensor_I2C/LSM6DSOX" -I"D:/STworkspace/EXP_V120/Dev/Sensor_I2C/BME280" -I"D:/STworkspace/EXP_V120/Dev/Sensor_I2C/BMP390" -I"D:/STworkspace/EXP_V120/Dev/Sensor_I2C/H250" -I"D:/STworkspace/EXP_V120/Dev/Sensor_I2C/H3LIS331DL" -I"D:/STworkspace/EXP_V120/Dev/Sensor_I2C/K33" -I"D:/STworkspace/EXP_V120/Dev/CLI_Terminal/CLI_Setup" -I"D:/STworkspace/EXP_V120/Dev/CLI_Terminal/CLI_Command" -I"D:/STworkspace/EXP_V120/Dev/CLI_Terminal/CLI_Src" -I"D:/STworkspace/EXP_V120/Dev/MIN_Proto/min" -I"D:/STworkspace/EXP_V120/Dev/MIN_Proto/min_app" -I"D:/STworkspace/EXP_V120/Dev/MIN_Proto" -I"D:/STworkspace/EXP_V120/Dev/Devices/TEST_LaserPhoto/ADG1414" -I"D:/STworkspace/EXP_V120/Dev/Devices/TEST_LaserPhoto/ADS8327" -I"D:/STworkspace/EXP_V120/Dev/Devices/TEST_LaserPhoto/MCP4902" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Dev-2f-Temperature-2f-Heater

clean-Dev-2f-Temperature-2f-Heater:
	-$(RM) ./Dev/Temperature/Heater/heater.cyclo ./Dev/Temperature/Heater/heater.d ./Dev/Temperature/Heater/heater.o ./Dev/Temperature/Heater/heater.su

.PHONY: clean-Dev-2f-Temperature-2f-Heater

