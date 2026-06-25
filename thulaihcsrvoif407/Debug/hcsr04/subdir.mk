################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hcsr04/hcsr04.c 

OBJS += \
./hcsr04/hcsr04.o 

C_DEPS += \
./hcsr04/hcsr04.d 


# Each subdirectory must supply rules for building sources it contributes
hcsr04/%.o hcsr04/%.su hcsr04/%.cyclo: ../hcsr04/%.c hcsr04/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/VScode/thulaihcsrvoif407/hcsr04" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-hcsr04

clean-hcsr04:
	-$(RM) ./hcsr04/hcsr04.cyclo ./hcsr04/hcsr04.d ./hcsr04/hcsr04.o ./hcsr04/hcsr04.su

.PHONY: clean-hcsr04

