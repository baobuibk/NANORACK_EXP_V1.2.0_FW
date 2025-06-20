################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Devices/BMP390/bmp390.c 

OBJS += \
./Core/Devices/BMP390/bmp390.o 

C_DEPS += \
./Core/Devices/BMP390/bmp390.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Devices/BMP390/%.o Core/Devices/BMP390/%.su Core/Devices/BMP390/%.cyclo: ../Core/Devices/BMP390/%.c Core/Devices/BMP390/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32F765xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=25000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=0 -DART_ACCELERATOR_ENABLE=0 -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/STworkspace/EXP_V120/Core/Board" -I"D:/STworkspace/EXP_V120/Scheduler" -I"D:/STworkspace/EXP_V120/Core/Devices/MB85RS2MT" -I"D:/STworkspace/EXP_V120/Core/Devices/NTC" -I"D:/STworkspace/EXP_V120/Core/Devices/PHOTO_BOARD" -I"D:/STworkspace/EXP_V120/Core/Devices/LT8722" -I"D:/STworkspace/EXP_V120/Core/Common" -I"D:/STworkspace/EXP_V120/BSP/UART" -I"D:/STworkspace/EXP_V120/BSP/I2C" -I"D:/STworkspace/EXP_V120/BSP/I2C_Slave" -I"D:/STworkspace/EXP_V120/BSP/SysTick" -I"D:/STworkspace/EXP_V120/Core/CLI_Terminal" -I"D:/STworkspace/EXP_V120/Core/Devices/ADG1414" -I"D:/STworkspace/EXP_V120/Core/Devices/Auto_run" -I"D:/STworkspace/EXP_V120/Core/Devices/BMP390" -I"D:/STworkspace/EXP_V120/Core/Devices/Heater" -I"D:/STworkspace/EXP_V120/Core/Devices/IR_LED" -I"D:/STworkspace/EXP_V120/Core/Devices/LASER_BOARD" -I"D:/STworkspace/EXP_V120/Core/LED" -I"D:/STworkspace/EXP_V120/Core/Sensor_I2C" -I"D:/STworkspace/EXP_V120/Core/Temperature" -I"D:/STworkspace/EXP_V120/Core/WDog" -I"D:/STworkspace/EXP_V120/Core/Devices/LASER_BOARD/MCP4902" -I"D:/STworkspace/EXP_V120/Core/Devices/PHOTO_BOARD/ADS8327" -I"D:/STworkspace/EXP_V120/Core/CLI_Terminal/CLI_Command" -I"D:/STworkspace/EXP_V120/Core/CLI_Terminal/CLI_Setup" -I"D:/STworkspace/EXP_V120/Core/CLI_Terminal/CLI_Src" -I"D:/STworkspace/EXP_V120/Core/Sensor_I2C/BME280" -I"D:/STworkspace/EXP_V120/Core/Sensor_I2C/BMP390" -I"D:/STworkspace/EXP_V120/Core/Sensor_I2C/H250" -I"D:/STworkspace/EXP_V120/Core/Sensor_I2C/H3LIS331DL" -I"D:/STworkspace/EXP_V120/Core/Sensor_I2C/K33" -I"D:/STworkspace/EXP_V120/Core/Sensor_I2C/LSM6DSOX" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Devices-2f-BMP390

clean-Core-2f-Devices-2f-BMP390:
	-$(RM) ./Core/Devices/BMP390/bmp390.cyclo ./Core/Devices/BMP390/bmp390.d ./Core/Devices/BMP390/bmp390.o ./Core/Devices/BMP390/bmp390.su

.PHONY: clean-Core-2f-Devices-2f-BMP390

