################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Dev/WDog/subdir.mk
-include Dev/Temperature/NTC/subdir.mk
-include Dev/Temperature/LT8722/subdir.mk
-include Dev/Temperature/Heater/subdir.mk
-include Dev/Temperature/subdir.mk
-include Dev/Sensor_I2C/LSM6DSOX/subdir.mk
-include Dev/Sensor_I2C/K33/subdir.mk
-include Dev/Sensor_I2C/H3LIS331DL/subdir.mk
-include Dev/Sensor_I2C/H250/subdir.mk
-include Dev/Sensor_I2C/BMP390/subdir.mk
-include Dev/Sensor_I2C/BME280/subdir.mk
-include Dev/Sensor_I2C/subdir.mk
-include Dev/Sche/subdir.mk
-include Dev/MiddleWare/sst0_c/src/subdir.mk
-include Dev/MiddleWare/container/subdir.mk
-include Dev/MiddleWare/Utils/subdir.mk
-include Dev/MiddleWare/STDIO/subdir.mk
-include Dev/MiddleWare/FSM/subdir.mk
-include Dev/MiddleWare/CLI_Terminal/CLI_Src/subdir.mk
-include Dev/MIN_Proto/min_app/subdir.mk
-include Dev/MIN_Proto/min/subdir.mk
-include Dev/MIN_Proto/subdir.mk
-include Dev/LED/subdir.mk
-include Dev/Devices/MCP4902/subdir.mk
-include Dev/Devices/MB85RS2MT/subdir.mk
-include Dev/Devices/IS66WVS4M8BLL/subdir.mk
-include Dev/Devices/IR_LED/subdir.mk
-include Dev/Devices/DateTime/subdir.mk
-include Dev/Devices/ADS8327/subdir.mk
-include Dev/Devices/ADG1414/subdir.mk
-include Dev/BSupport/UART/RingBuffer/subdir.mk
-include Dev/BSupport/UART/subdir.mk
-include Dev/BSupport/SysTick/subdir.mk
-include Dev/BSupport/I2C/subdir.mk
-include Dev/BSupport/BSP/bsp_tec/subdir.mk
-include Dev/BSupport/BSP/bsp_photodiode/subdir.mk
-include Dev/BSupport/BSP/bsp_ntc/subdir.mk
-include Dev/BSupport/BSP/bsp_heater/subdir.mk
-include Dev/BSupport/BSP/Board/subdir.mk
-include Dev/App/temperature_monitor/subdir.mk
-include Dev/App/temperature_control/subdir.mk
-include Dev/App/shell/CLI_Command/subdir.mk
-include Dev/App/shell/subdir.mk
-include Dev/App/app_main/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := EXP_V120
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
EXP_V120.elf \

MAP_FILES += \
EXP_V120.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
EXP_V120.list \

OBJCOPY_HEX += \
EXP_V120.hex \

OBJCOPY_BIN += \
EXP_V120.bin \


# All Target
all: main-build

# Main-build Target
main-build: EXP_V120.elf secondary-outputs

# Tool invocations
EXP_V120.elf EXP_V120.map: $(OBJS) $(USER_OBJS) D:\PROJECT\BOOK_EXAMPLE\EXP_V120\STM32F765VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "EXP_V120.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"D:\PROJECT\BOOK_EXAMPLE\EXP_V120\STM32F765VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="EXP_V120.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

EXP_V120.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "EXP_V120.list"
	@echo 'Finished building: $@'
	@echo ' '

EXP_V120.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "EXP_V120.hex"
	@echo 'Finished building: $@'
	@echo ' '

EXP_V120.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "EXP_V120.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) EXP_V120.bin EXP_V120.elf EXP_V120.hex EXP_V120.list EXP_V120.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
