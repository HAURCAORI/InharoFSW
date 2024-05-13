################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/module/buzzer.c 

OBJS += \
./Core/Src/module/buzzer.o 

C_DEPS += \
./Core/Src/module/buzzer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/module/%.o Core/Src/module/%.su Core/Src/module/%.cyclo: ../Core/Src/module/%.c Core/Src/module/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Core/ThreadSafe -I"C:/Users/SURFACE/STM32CubeIDE/workspace_1.12.0/InharoFSW/Inharo_fsw/Core/Inc/initialize" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-module

clean-Core-2f-Src-2f-module:
	-$(RM) ./Core/Src/module/buzzer.cyclo ./Core/Src/module/buzzer.d ./Core/Src/module/buzzer.o ./Core/Src/module/buzzer.su

.PHONY: clean-Core-2f-Src-2f-module

