################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Scheduler/scheduler.c 

OBJS += \
./Scheduler/scheduler.o 

C_DEPS += \
./Scheduler/scheduler.d 


# Each subdirectory must supply rules for building sources it contributes
Scheduler/%.o Scheduler/%.su Scheduler/%.cyclo: ../Scheduler/%.c Scheduler/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32F765xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/STworkspace/EXP_V120/Scheduler" -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Scheduler

clean-Scheduler:
	-$(RM) ./Scheduler/scheduler.cyclo ./Scheduler/scheduler.d ./Scheduler/scheduler.o ./Scheduler/scheduler.su

.PHONY: clean-Scheduler

