################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/PCF2131/PCF2131.c 

OBJS += \
./Drivers/PCF2131/PCF2131.o 

C_DEPS += \
./Drivers/PCF2131/PCF2131.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/PCF2131/%.o Drivers/PCF2131/%.su Drivers/PCF2131/%.cyclo: ../Drivers/PCF2131/%.c Drivers/PCF2131/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../Core/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-PCF2131

clean-Drivers-2f-PCF2131:
	-$(RM) ./Drivers/PCF2131/PCF2131.cyclo ./Drivers/PCF2131/PCF2131.d ./Drivers/PCF2131/PCF2131.o ./Drivers/PCF2131/PCF2131.su

.PHONY: clean-Drivers-2f-PCF2131

