################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/bsp_driver_sd.c \
../Src/debug.c \
../Src/dma.c \
../Src/fatfs.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/lptim.c \
../Src/main.c \
../Src/memorymap.c \
../Src/rtc.c \
../Src/sd_diskio.c \
../Src/sdmmc.c \
../Src/spi.c \
../Src/stm32h7xx_hal_msp.c \
../Src/stm32h7xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32h7xx.c \
../Src/tim.c 

OBJS += \
./Src/adc.o \
./Src/bsp_driver_sd.o \
./Src/debug.o \
./Src/dma.o \
./Src/fatfs.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/lptim.o \
./Src/main.o \
./Src/memorymap.o \
./Src/rtc.o \
./Src/sd_diskio.o \
./Src/sdmmc.o \
./Src/spi.o \
./Src/stm32h7xx_hal_msp.o \
./Src/stm32h7xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32h7xx.o \
./Src/tim.o 

C_DEPS += \
./Src/adc.d \
./Src/bsp_driver_sd.d \
./Src/debug.d \
./Src/dma.d \
./Src/fatfs.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/lptim.d \
./Src/main.d \
./Src/memorymap.d \
./Src/rtc.d \
./Src/sd_diskio.d \
./Src/sdmmc.d \
./Src/spi.d \
./Src/stm32h7xx_hal_msp.d \
./Src/stm32h7xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32h7xx.d \
./Src/tim.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I../Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kth59/OneDrive/Desktop/WEACT/WEACT 743 V2/Src" -I"C:/Users/kth59/OneDrive/Desktop/WEACT/WEACT 743 V2/Drivers/BSP/ST7735" -I"C:/Users/kth59/OneDrive/Desktop/WEACT/WEACT 743 V2/Main/Inc" -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/adc.cyclo ./Src/adc.d ./Src/adc.o ./Src/adc.su ./Src/bsp_driver_sd.cyclo ./Src/bsp_driver_sd.d ./Src/bsp_driver_sd.o ./Src/bsp_driver_sd.su ./Src/debug.cyclo ./Src/debug.d ./Src/debug.o ./Src/debug.su ./Src/dma.cyclo ./Src/dma.d ./Src/dma.o ./Src/dma.su ./Src/fatfs.cyclo ./Src/fatfs.d ./Src/fatfs.o ./Src/fatfs.su ./Src/gpio.cyclo ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/i2c.cyclo ./Src/i2c.d ./Src/i2c.o ./Src/i2c.su ./Src/lptim.cyclo ./Src/lptim.d ./Src/lptim.o ./Src/lptim.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/memorymap.cyclo ./Src/memorymap.d ./Src/memorymap.o ./Src/memorymap.su ./Src/rtc.cyclo ./Src/rtc.d ./Src/rtc.o ./Src/rtc.su ./Src/sd_diskio.cyclo ./Src/sd_diskio.d ./Src/sd_diskio.o ./Src/sd_diskio.su ./Src/sdmmc.cyclo ./Src/sdmmc.d ./Src/sdmmc.o ./Src/sdmmc.su ./Src/spi.cyclo ./Src/spi.d ./Src/spi.o ./Src/spi.su ./Src/stm32h7xx_hal_msp.cyclo ./Src/stm32h7xx_hal_msp.d ./Src/stm32h7xx_hal_msp.o ./Src/stm32h7xx_hal_msp.su ./Src/stm32h7xx_it.cyclo ./Src/stm32h7xx_it.d ./Src/stm32h7xx_it.o ./Src/stm32h7xx_it.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32h7xx.cyclo ./Src/system_stm32h7xx.d ./Src/system_stm32h7xx.o ./Src/system_stm32h7xx.su ./Src/tim.cyclo ./Src/tim.d ./Src/tim.o ./Src/tim.su

.PHONY: clean-Src

