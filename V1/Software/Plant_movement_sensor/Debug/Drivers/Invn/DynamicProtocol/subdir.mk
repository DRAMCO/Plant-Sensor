################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Invn/DynamicProtocol/DynProtocol.c \
../Drivers/Invn/DynamicProtocol/DynProtocolTransportUart.c 

OBJS += \
./Drivers/Invn/DynamicProtocol/DynProtocol.o \
./Drivers/Invn/DynamicProtocol/DynProtocolTransportUart.o 

C_DEPS += \
./Drivers/Invn/DynamicProtocol/DynProtocol.d \
./Drivers/Invn/DynamicProtocol/DynProtocolTransportUart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Invn/DynamicProtocol/%.o Drivers/Invn/DynamicProtocol/%.su Drivers/Invn/DynamicProtocol/%.cyclo: ../Drivers/Invn/DynamicProtocol/%.c Drivers/Invn/DynamicProtocol/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../Core/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Invn-2f-DynamicProtocol

clean-Drivers-2f-Invn-2f-DynamicProtocol:
	-$(RM) ./Drivers/Invn/DynamicProtocol/DynProtocol.cyclo ./Drivers/Invn/DynamicProtocol/DynProtocol.d ./Drivers/Invn/DynamicProtocol/DynProtocol.o ./Drivers/Invn/DynamicProtocol/DynProtocol.su ./Drivers/Invn/DynamicProtocol/DynProtocolTransportUart.cyclo ./Drivers/Invn/DynamicProtocol/DynProtocolTransportUart.d ./Drivers/Invn/DynamicProtocol/DynProtocolTransportUart.o ./Drivers/Invn/DynamicProtocol/DynProtocolTransportUart.su

.PHONY: clean-Drivers-2f-Invn-2f-DynamicProtocol

