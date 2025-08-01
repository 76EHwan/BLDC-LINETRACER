################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/ST7735/lcd.c \
../Drivers/BSP/ST7735/logo_128_160.c \
../Drivers/BSP/ST7735/logo_160_80.c \
../Drivers/BSP/ST7735/st7735.c \
../Drivers/BSP/ST7735/st7735_reg.c 

OBJS += \
./Drivers/BSP/ST7735/lcd.o \
./Drivers/BSP/ST7735/logo_128_160.o \
./Drivers/BSP/ST7735/logo_160_80.o \
./Drivers/BSP/ST7735/st7735.o \
./Drivers/BSP/ST7735/st7735_reg.o 

C_DEPS += \
./Drivers/BSP/ST7735/lcd.d \
./Drivers/BSP/ST7735/logo_128_160.d \
./Drivers/BSP/ST7735/logo_160_80.d \
./Drivers/BSP/ST7735/st7735.d \
./Drivers/BSP/ST7735/st7735_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/ST7735/%.o Drivers/BSP/ST7735/%.su Drivers/BSP/ST7735/%.cyclo: ../Drivers/BSP/ST7735/%.c Drivers/BSP/ST7735/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I../Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kth59/OneDrive/Desktop/WEACT/WEACT 743 V2/Src" -I"C:/Users/kth59/OneDrive/Desktop/WEACT/WEACT 743 V2/Drivers/BSP/ST7735" -I"C:/Users/kth59/OneDrive/Desktop/WEACT/WEACT 743 V2/Main/Inc" -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-ST7735

clean-Drivers-2f-BSP-2f-ST7735:
	-$(RM) ./Drivers/BSP/ST7735/lcd.cyclo ./Drivers/BSP/ST7735/lcd.d ./Drivers/BSP/ST7735/lcd.o ./Drivers/BSP/ST7735/lcd.su ./Drivers/BSP/ST7735/logo_128_160.cyclo ./Drivers/BSP/ST7735/logo_128_160.d ./Drivers/BSP/ST7735/logo_128_160.o ./Drivers/BSP/ST7735/logo_128_160.su ./Drivers/BSP/ST7735/logo_160_80.cyclo ./Drivers/BSP/ST7735/logo_160_80.d ./Drivers/BSP/ST7735/logo_160_80.o ./Drivers/BSP/ST7735/logo_160_80.su ./Drivers/BSP/ST7735/st7735.cyclo ./Drivers/BSP/ST7735/st7735.d ./Drivers/BSP/ST7735/st7735.o ./Drivers/BSP/ST7735/st7735.su ./Drivers/BSP/ST7735/st7735_reg.cyclo ./Drivers/BSP/ST7735/st7735_reg.d ./Drivers/BSP/ST7735/st7735_reg.o ./Drivers/BSP/ST7735/st7735_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-ST7735

