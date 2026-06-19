################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/OLED/Src/ssd1306.c \
../Drivers/OLED/Src/ssd1306_fonts.c \
../Drivers/OLED/Src/ssd1306_tests.c 

OBJS += \
./Drivers/OLED/Src/ssd1306.o \
./Drivers/OLED/Src/ssd1306_fonts.o \
./Drivers/OLED/Src/ssd1306_tests.o 

C_DEPS += \
./Drivers/OLED/Src/ssd1306.d \
./Drivers/OLED/Src/ssd1306_fonts.d \
./Drivers/OLED/Src/ssd1306_tests.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/OLED/Src/%.o Drivers/OLED/Src/%.su Drivers/OLED/Src/%.cyclo: ../Drivers/OLED/Src/%.c Drivers/OLED/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"D:/feature-hardware/Robot_hut_bui/Drivers/OLED/Inc" -I"D:/feature-hardware/Robot_hut_bui/Drivers/OLED/Src" -I"E:/STM32f4/doan_01/hardware/inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-OLED-2f-Src

clean-Drivers-2f-OLED-2f-Src:
	-$(RM) ./Drivers/OLED/Src/ssd1306.cyclo ./Drivers/OLED/Src/ssd1306.d ./Drivers/OLED/Src/ssd1306.o ./Drivers/OLED/Src/ssd1306.su ./Drivers/OLED/Src/ssd1306_fonts.cyclo ./Drivers/OLED/Src/ssd1306_fonts.d ./Drivers/OLED/Src/ssd1306_fonts.o ./Drivers/OLED/Src/ssd1306_fonts.su ./Drivers/OLED/Src/ssd1306_tests.cyclo ./Drivers/OLED/Src/ssd1306_tests.d ./Drivers/OLED/Src/ssd1306_tests.o ./Drivers/OLED/Src/ssd1306_tests.su

.PHONY: clean-Drivers-2f-OLED-2f-Src

