################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/initialize/adc.c \
../Core/Src/initialize/clock.c \
../Core/Src/initialize/gpio.c \
../Core/Src/initialize/i2c.c \
../Core/Src/initialize/rtc.c \
../Core/Src/initialize/spi.c \
../Core/Src/initialize/tim.c \
../Core/Src/initialize/usart.c 

OBJS += \
./Core/Src/initialize/adc.o \
./Core/Src/initialize/clock.o \
./Core/Src/initialize/gpio.o \
./Core/Src/initialize/i2c.o \
./Core/Src/initialize/rtc.o \
./Core/Src/initialize/spi.o \
./Core/Src/initialize/tim.o \
./Core/Src/initialize/usart.o 

C_DEPS += \
./Core/Src/initialize/adc.d \
./Core/Src/initialize/clock.d \
./Core/Src/initialize/gpio.d \
./Core/Src/initialize/i2c.d \
./Core/Src/initialize/rtc.d \
./Core/Src/initialize/spi.d \
./Core/Src/initialize/tim.d \
./Core/Src/initialize/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/initialize/%.o Core/Src/initialize/%.su Core/Src/initialize/%.cyclo: ../Core/Src/initialize/%.c Core/Src/initialize/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Core/ThreadSafe -I"C:/Users/SURFACE/STM32CubeIDE/workspace_1.12.0/InharoFSW/Inharo_fsw/Core/Inc/initialize" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-initialize

clean-Core-2f-Src-2f-initialize:
	-$(RM) ./Core/Src/initialize/adc.cyclo ./Core/Src/initialize/adc.d ./Core/Src/initialize/adc.o ./Core/Src/initialize/adc.su ./Core/Src/initialize/clock.cyclo ./Core/Src/initialize/clock.d ./Core/Src/initialize/clock.o ./Core/Src/initialize/clock.su ./Core/Src/initialize/gpio.cyclo ./Core/Src/initialize/gpio.d ./Core/Src/initialize/gpio.o ./Core/Src/initialize/gpio.su ./Core/Src/initialize/i2c.cyclo ./Core/Src/initialize/i2c.d ./Core/Src/initialize/i2c.o ./Core/Src/initialize/i2c.su ./Core/Src/initialize/rtc.cyclo ./Core/Src/initialize/rtc.d ./Core/Src/initialize/rtc.o ./Core/Src/initialize/rtc.su ./Core/Src/initialize/spi.cyclo ./Core/Src/initialize/spi.d ./Core/Src/initialize/spi.o ./Core/Src/initialize/spi.su ./Core/Src/initialize/tim.cyclo ./Core/Src/initialize/tim.d ./Core/Src/initialize/tim.o ./Core/Src/initialize/tim.su ./Core/Src/initialize/usart.cyclo ./Core/Src/initialize/usart.d ./Core/Src/initialize/usart.o ./Core/Src/initialize/usart.su

.PHONY: clean-Core-2f-Src-2f-initialize

