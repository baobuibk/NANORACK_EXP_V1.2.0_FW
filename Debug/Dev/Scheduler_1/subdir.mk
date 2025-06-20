################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Dev/Scheduler_1/scheduler.c 

OBJS += \
./Dev/Scheduler_1/scheduler.o 

C_DEPS += \
./Dev/Scheduler_1/scheduler.d 


# Each subdirectory must supply rules for building sources it contributes
Dev/Scheduler_1/%.o Dev/Scheduler_1/%.su Dev/Scheduler_1/%.cyclo: ../Dev/Scheduler_1/%.c Dev/Scheduler_1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32F765xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Dev-2f-Scheduler_1

clean-Dev-2f-Scheduler_1:
	-$(RM) ./Dev/Scheduler_1/scheduler.cyclo ./Dev/Scheduler_1/scheduler.d ./Dev/Scheduler_1/scheduler.o ./Dev/Scheduler_1/scheduler.su

.PHONY: clean-Dev-2f-Scheduler_1

