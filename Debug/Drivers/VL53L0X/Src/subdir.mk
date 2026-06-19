################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L0X/Src/VL53L0X.c 

OBJS += \
./Drivers/VL53L0X/Src/VL53L0X.o 

C_DEPS += \
./Drivers/VL53L0X/Src/VL53L0X.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L0X/Src/%.o Drivers/VL53L0X/Src/%.su Drivers/VL53L0X/Src/%.cyclo: ../Drivers/VL53L0X/Src/%.c Drivers/VL53L0X/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"D:/feature-hardware/Robot_hut_bui/Drivers/OLED/Inc" -I"D:/feature-hardware/Robot_hut_bui/Drivers/OLED/Src" -I"E:/STM32f4/doan_01/hardware/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L0X-2f-Src

clean-Drivers-2f-VL53L0X-2f-Src:
	-$(RM) ./Drivers/VL53L0X/Src/VL53L0X.cyclo ./Drivers/VL53L0X/Src/VL53L0X.d ./Drivers/VL53L0X/Src/VL53L0X.o ./Drivers/VL53L0X/Src/VL53L0X.su

.PHONY: clean-Drivers-2f-VL53L0X-2f-Src

