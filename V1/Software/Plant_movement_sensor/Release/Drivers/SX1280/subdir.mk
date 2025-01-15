################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SX1280/SX1280.c 

OBJS += \
./Drivers/SX1280/SX1280.o 

C_DEPS += \
./Drivers/SX1280/SX1280.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SX1280/%.o Drivers/SX1280/%.su Drivers/SX1280/%.cyclo: ../Drivers/SX1280/%.c Drivers/SX1280/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../Core/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SX1280

clean-Drivers-2f-SX1280:
	-$(RM) ./Drivers/SX1280/SX1280.cyclo ./Drivers/SX1280/SX1280.d ./Drivers/SX1280/SX1280.o ./Drivers/SX1280/SX1280.su

.PHONY: clean-Drivers-2f-SX1280

