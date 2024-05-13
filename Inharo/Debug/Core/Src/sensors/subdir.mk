################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/sensors/bmp390.c \
../Core/Src/sensors/bno055.c \
../Core/Src/sensors/bno055_stm32.c \
../Core/Src/sensors/init_sensor.c 

OBJS += \
./Core/Src/sensors/bmp390.o \
./Core/Src/sensors/bno055.o \
./Core/Src/sensors/bno055_stm32.o \
./Core/Src/sensors/init_sensor.o 

C_DEPS += \
./Core/Src/sensors/bmp390.d \
./Core/Src/sensors/bno055.d \
./Core/Src/sensors/bno055_stm32.d \
./Core/Src/sensors/init_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sensors/%.o Core/Src/sensors/%.su Core/Src/sensors/%.cyclo: ../Core/Src/sensors/%.c Core/Src/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I"C:/Project/STM32/Inharo/Core/Inc/initialize" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Core/ThreadSafe -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-sensors

clean-Core-2f-Src-2f-sensors:
	-$(RM) ./Core/Src/sensors/bmp390.cyclo ./Core/Src/sensors/bmp390.d ./Core/Src/sensors/bmp390.o ./Core/Src/sensors/bmp390.su ./Core/Src/sensors/bno055.cyclo ./Core/Src/sensors/bno055.d ./Core/Src/sensors/bno055.o ./Core/Src/sensors/bno055.su ./Core/Src/sensors/bno055_stm32.cyclo ./Core/Src/sensors/bno055_stm32.d ./Core/Src/sensors/bno055_stm32.o ./Core/Src/sensors/bno055_stm32.su ./Core/Src/sensors/init_sensor.cyclo ./Core/Src/sensors/init_sensor.d ./Core/Src/sensors/init_sensor.o ./Core/Src/sensors/init_sensor.su

.PHONY: clean-Core-2f-Src-2f-sensors

