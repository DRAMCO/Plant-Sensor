################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Invn/VSensor/VSensor.c 

OBJS += \
./Drivers/Invn/VSensor/VSensor.o 

C_DEPS += \
./Drivers/Invn/VSensor/VSensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Invn/VSensor/%.o Drivers/Invn/VSensor/%.su Drivers/Invn/VSensor/%.cyclo: ../Drivers/Invn/VSensor/%.c Drivers/Invn/VSensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../Core/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Invn-2f-VSensor

clean-Drivers-2f-Invn-2f-VSensor:
	-$(RM) ./Drivers/Invn/VSensor/VSensor.cyclo ./Drivers/Invn/VSensor/VSensor.d ./Drivers/Invn/VSensor/VSensor.o ./Drivers/Invn/VSensor/VSensor.su

.PHONY: clean-Drivers-2f-Invn-2f-VSensor

