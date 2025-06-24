################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Dev/MiddleWare/sst0_c/examples/blinky_button/blinky1.c \
../Dev/MiddleWare/sst0_c/examples/blinky_button/blinky3.c \
../Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_ek-tm4c123gxl.c \
../Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-c031c6.c \
../Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-h743zi.c \
../Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-l053r8.c \
../Dev/MiddleWare/sst0_c/examples/blinky_button/button2a.c \
../Dev/MiddleWare/sst0_c/examples/blinky_button/button2b.c \
../Dev/MiddleWare/sst0_c/examples/blinky_button/main.c 

OBJS += \
./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky1.o \
./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky3.o \
./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_ek-tm4c123gxl.o \
./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-c031c6.o \
./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-h743zi.o \
./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-l053r8.o \
./Dev/MiddleWare/sst0_c/examples/blinky_button/button2a.o \
./Dev/MiddleWare/sst0_c/examples/blinky_button/button2b.o \
./Dev/MiddleWare/sst0_c/examples/blinky_button/main.o 

C_DEPS += \
./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky1.d \
./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky3.d \
./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_ek-tm4c123gxl.d \
./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-c031c6.d \
./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-h743zi.d \
./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-l053r8.d \
./Dev/MiddleWare/sst0_c/examples/blinky_button/button2a.d \
./Dev/MiddleWare/sst0_c/examples/blinky_button/button2b.d \
./Dev/MiddleWare/sst0_c/examples/blinky_button/main.d 


# Each subdirectory must supply rules for building sources it contributes
Dev/MiddleWare/sst0_c/examples/blinky_button/%.o Dev/MiddleWare/sst0_c/examples/blinky_button/%.su Dev/MiddleWare/sst0_c/examples/blinky_button/%.cyclo: ../Dev/MiddleWare/sst0_c/examples/blinky_button/%.c Dev/MiddleWare/sst0_c/examples/blinky_button/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32F765xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/WDog" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Board" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/CLI_Terminal" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Common" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/LED" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sche" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/UART" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/SysTick" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/BSupport/I2C" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/Auto_run" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/DateTime" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/IR_LED" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/MB85RS2MT" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature/LT8722" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature/Heater" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Temperature/NTC" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/LSM6DSOX" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/BME280" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/BMP390" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/H250" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/H3LIS331DL" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Sensor_I2C/K33" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/CLI_Terminal/CLI_Setup" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/CLI_Terminal/CLI_Command" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/CLI_Terminal/CLI_Src" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto/min" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto/min_app" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/MIN_Proto" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/TEST_LaserPhoto/ADG1414" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/TEST_LaserPhoto/ADS8327" -I"D:/PROJECT/BOOK_EXAMPLE/EXP_V120/Dev/Devices/TEST_LaserPhoto/MCP4902" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Dev-2f-MiddleWare-2f-sst0_c-2f-examples-2f-blinky_button

clean-Dev-2f-MiddleWare-2f-sst0_c-2f-examples-2f-blinky_button:
	-$(RM) ./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky1.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky1.d ./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky1.o ./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky1.su ./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky3.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky3.d ./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky3.o ./Dev/MiddleWare/sst0_c/examples/blinky_button/blinky3.su ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_ek-tm4c123gxl.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_ek-tm4c123gxl.d ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_ek-tm4c123gxl.o ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_ek-tm4c123gxl.su ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-c031c6.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-c031c6.d ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-c031c6.o ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-c031c6.su ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-h743zi.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-h743zi.d ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-h743zi.o ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-h743zi.su ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-l053r8.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-l053r8.d ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-l053r8.o ./Dev/MiddleWare/sst0_c/examples/blinky_button/bsp_nucleo-l053r8.su ./Dev/MiddleWare/sst0_c/examples/blinky_button/button2a.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky_button/button2a.d ./Dev/MiddleWare/sst0_c/examples/blinky_button/button2a.o ./Dev/MiddleWare/sst0_c/examples/blinky_button/button2a.su ./Dev/MiddleWare/sst0_c/examples/blinky_button/button2b.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky_button/button2b.d ./Dev/MiddleWare/sst0_c/examples/blinky_button/button2b.o ./Dev/MiddleWare/sst0_c/examples/blinky_button/button2b.su ./Dev/MiddleWare/sst0_c/examples/blinky_button/main.cyclo ./Dev/MiddleWare/sst0_c/examples/blinky_button/main.d ./Dev/MiddleWare/sst0_c/examples/blinky_button/main.o ./Dev/MiddleWare/sst0_c/examples/blinky_button/main.su

.PHONY: clean-Dev-2f-MiddleWare-2f-sst0_c-2f-examples-2f-blinky_button

