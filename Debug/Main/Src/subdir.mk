################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Main/Src/custom_delay.c \
../Main/Src/drive.c \
../Main/Src/init.c \
../Main/Src/lsm6ds3tr_c.c \
../Main/Src/mcf8316c.c \
../Main/Src/motor.c \
../Main/Src/sdcard.c \
../Main/Src/sensor.c \
../Main/Src/switch.c 

OBJS += \
./Main/Src/custom_delay.o \
./Main/Src/drive.o \
./Main/Src/init.o \
./Main/Src/lsm6ds3tr_c.o \
./Main/Src/mcf8316c.o \
./Main/Src/motor.o \
./Main/Src/sdcard.o \
./Main/Src/sensor.o \
./Main/Src/switch.o 

C_DEPS += \
./Main/Src/custom_delay.d \
./Main/Src/drive.d \
./Main/Src/init.d \
./Main/Src/lsm6ds3tr_c.d \
./Main/Src/mcf8316c.d \
./Main/Src/motor.d \
./Main/Src/sdcard.d \
./Main/Src/sensor.d \
./Main/Src/switch.d 


# Each subdirectory must supply rules for building sources it contributes
Main/Src/%.o Main/Src/%.su Main/Src/%.cyclo: ../Main/Src/%.c Main/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I../Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kth59/OneDrive/Desktop/WEACT/WEACT 743 V2/Src" -I"C:/Users/kth59/OneDrive/Desktop/WEACT/WEACT 743 V2/Drivers/BSP/ST7735" -I"C:/Users/kth59/OneDrive/Desktop/WEACT/WEACT 743 V2/Main/Inc" -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Main-2f-Src

clean-Main-2f-Src:
	-$(RM) ./Main/Src/custom_delay.cyclo ./Main/Src/custom_delay.d ./Main/Src/custom_delay.o ./Main/Src/custom_delay.su ./Main/Src/drive.cyclo ./Main/Src/drive.d ./Main/Src/drive.o ./Main/Src/drive.su ./Main/Src/init.cyclo ./Main/Src/init.d ./Main/Src/init.o ./Main/Src/init.su ./Main/Src/lsm6ds3tr_c.cyclo ./Main/Src/lsm6ds3tr_c.d ./Main/Src/lsm6ds3tr_c.o ./Main/Src/lsm6ds3tr_c.su ./Main/Src/mcf8316c.cyclo ./Main/Src/mcf8316c.d ./Main/Src/mcf8316c.o ./Main/Src/mcf8316c.su ./Main/Src/motor.cyclo ./Main/Src/motor.d ./Main/Src/motor.o ./Main/Src/motor.su ./Main/Src/sdcard.cyclo ./Main/Src/sdcard.d ./Main/Src/sdcard.o ./Main/Src/sdcard.su ./Main/Src/sensor.cyclo ./Main/Src/sensor.d ./Main/Src/sensor.o ./Main/Src/sensor.su ./Main/Src/switch.cyclo ./Main/Src/switch.d ./Main/Src/switch.o ./Main/Src/switch.su

.PHONY: clean-Main-2f-Src

