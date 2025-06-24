################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Dev/MiddleWare/sst0_c/examples/blinky/blinky.c \
../Dev/MiddleWare/sst0_c/examples/blinky/bsp_nucleo-l053r8.c \
../Dev/MiddleWare/sst0_c/examples/blinky/main.c 

OBJS += \
./Dev/MiddleWare/sst0_c/examples/blinky/blinky.o \
./Dev/MiddleWare/sst0_c/examples/blinky/bsp_nucleo-l053r8.o \
./Dev/MiddleWare/sst0_c/examples/blinky/main.o 

C_DEPS += \
./Dev/MiddleWare/sst0_c/examples/blinky/blinky.d \
./Dev/MiddleWare/sst0_c/examples/blinky/bsp_nucleo-l053r8.d \
./Dev/MiddleWare/sst0_c/examples/blinky/main.d 


# Each subdirectory must supply rules for building sources it contributes
Dev/MiddleWare/sst0_c/examples/blinky/%.o Dev/MiddleWare/sst0_c/examples/blinky/%.su Dev/MiddleWare/sst0_c/examples/blinky/%.cyclo: ../Dev/MiddleWare/sst0_c/examples/blinky/%.c Dev/MiddleWare/sst0_c/examples/blinky/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32F765xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/WDog" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Board" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/CLI_Terminal" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Common" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/LED" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sche" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/UART" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/SysTick" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/I2C" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/Auto_run" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/DateTime" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/IR_LED" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/MB85RS2MT" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature/LT8722" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature/Heater" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature/NTC" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/LSM6DSOX" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/BME280" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/BMP390" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/H250" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/H3LIS331DL" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/K33" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/CLI_Terminal/CLI_Setup" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/CLI_Terminal/CLI_Command" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/CLI_Terminal/CLI_Src" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto/min" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto/min_app" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/TEST_LaserPhoto/ADG1414" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/TEST_LaserPhoto/ADS8327" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/TEST_LaserPhoto/MCP4902" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Dev-2f-MiddleWare-2f-sst0_c-2f-examples-2f-blinky

clean-Dev-2f-MiddleWare-2f-sst0_c-2f-examples-2f-blinky:
	-$(RM) ./Dev/MiddleWare/sst0_c/examples/blinky/blinky.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky/blinky.d ./Dev/MiddleWare/sst0_c/examples/blinky/blinky.o ./Dev/MiddleWare/sst0_c/examples/blinky/blinky.su ./Dev/MiddleWare/sst0_c/examples/blinky/bsp_nucleo-l053r8.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky/bsp_nucleo-l053r8.d ./Dev/MiddleWare/sst0_c/examples/blinky/bsp_nucleo-l053r8.o ./Dev/MiddleWare/sst0_c/examples/blinky/bsp_nucleo-l053r8.su ./Dev/MiddleWare/sst0_c/examples/blinky/main.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky/main.d ./Dev/MiddleWare/sst0_c/examples/blinky/main.o ./Dev/MiddleWare/sst0_c/examples/blinky/main.su

.PHONY: clean-Dev-2f-MiddleWare-2f-sst0_c-2f-examples-2f-blinky

