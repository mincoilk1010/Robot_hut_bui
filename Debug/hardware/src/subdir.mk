################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hardware/src/Servo.c \
../hardware/src/VL53L0X.c \
../hardware/src/control.c \
../hardware/src/delay.c \
../hardware/src/encoder.c \
../hardware/src/hc_sr04.c \
../hardware/src/map.c \
../hardware/src/motor.c \
../hardware/src/mpu6050.c \
../hardware/src/pid.c 

OBJS += \
./hardware/src/Servo.o \
./hardware/src/VL53L0X.o \
./hardware/src/control.o \
./hardware/src/delay.o \
./hardware/src/encoder.o \
./hardware/src/hc_sr04.o \
./hardware/src/map.o \
./hardware/src/motor.o \
./hardware/src/mpu6050.o \
./hardware/src/pid.o 

C_DEPS += \
./hardware/src/Servo.d \
./hardware/src/VL53L0X.d \
./hardware/src/control.d \
./hardware/src/delay.d \
./hardware/src/encoder.d \
./hardware/src/hc_sr04.d \
./hardware/src/map.d \
./hardware/src/motor.d \
./hardware/src/mpu6050.d \
./hardware/src/pid.d 


# Each subdirectory must supply rules for building sources it contributes
hardware/src/%.o hardware/src/%.su hardware/src/%.cyclo: ../hardware/src/%.c hardware/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"E:/STM32f4/doan_01/hardware/src" -I"E:/STM32f4/doan_01/hardware/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-hardware-2f-src

clean-hardware-2f-src:
	-$(RM) ./hardware/src/Servo.cyclo ./hardware/src/Servo.d ./hardware/src/Servo.o ./hardware/src/Servo.su ./hardware/src/VL53L0X.cyclo ./hardware/src/VL53L0X.d ./hardware/src/VL53L0X.o ./hardware/src/VL53L0X.su ./hardware/src/control.cyclo ./hardware/src/control.d ./hardware/src/control.o ./hardware/src/control.su ./hardware/src/delay.cyclo ./hardware/src/delay.d ./hardware/src/delay.o ./hardware/src/delay.su ./hardware/src/encoder.cyclo ./hardware/src/encoder.d ./hardware/src/encoder.o ./hardware/src/encoder.su ./hardware/src/hc_sr04.cyclo ./hardware/src/hc_sr04.d ./hardware/src/hc_sr04.o ./hardware/src/hc_sr04.su ./hardware/src/map.cyclo ./hardware/src/map.d ./hardware/src/map.o ./hardware/src/map.su ./hardware/src/motor.cyclo ./hardware/src/motor.d ./hardware/src/motor.o ./hardware/src/motor.su ./hardware/src/mpu6050.cyclo ./hardware/src/mpu6050.d ./hardware/src/mpu6050.o ./hardware/src/mpu6050.su ./hardware/src/pid.cyclo ./hardware/src/pid.d ./hardware/src/pid.o ./hardware/src/pid.su

.PHONY: clean-hardware-2f-src

