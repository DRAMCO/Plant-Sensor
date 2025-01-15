################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Invn/EmbUtils/DataConverter.c \
../Drivers/Invn/EmbUtils/ErrorHelper.c \
../Drivers/Invn/EmbUtils/InvBasicMath.c \
../Drivers/Invn/EmbUtils/InvCksum.c \
../Drivers/Invn/EmbUtils/InvFormat.c \
../Drivers/Invn/EmbUtils/InvList.c \
../Drivers/Invn/EmbUtils/InvPrintf.c \
../Drivers/Invn/EmbUtils/InvProtocol.c \
../Drivers/Invn/EmbUtils/InvQueue.c \
../Drivers/Invn/EmbUtils/InvScheduler.c \
../Drivers/Invn/EmbUtils/Logger.c \
../Drivers/Invn/EmbUtils/Message.c \
../Drivers/Invn/EmbUtils/RingByteBuffer.c \
../Drivers/Invn/EmbUtils/UartTxEmulator.c 

OBJS += \
./Drivers/Invn/EmbUtils/DataConverter.o \
./Drivers/Invn/EmbUtils/ErrorHelper.o \
./Drivers/Invn/EmbUtils/InvBasicMath.o \
./Drivers/Invn/EmbUtils/InvCksum.o \
./Drivers/Invn/EmbUtils/InvFormat.o \
./Drivers/Invn/EmbUtils/InvList.o \
./Drivers/Invn/EmbUtils/InvPrintf.o \
./Drivers/Invn/EmbUtils/InvProtocol.o \
./Drivers/Invn/EmbUtils/InvQueue.o \
./Drivers/Invn/EmbUtils/InvScheduler.o \
./Drivers/Invn/EmbUtils/Logger.o \
./Drivers/Invn/EmbUtils/Message.o \
./Drivers/Invn/EmbUtils/RingByteBuffer.o \
./Drivers/Invn/EmbUtils/UartTxEmulator.o 

C_DEPS += \
./Drivers/Invn/EmbUtils/DataConverter.d \
./Drivers/Invn/EmbUtils/ErrorHelper.d \
./Drivers/Invn/EmbUtils/InvBasicMath.d \
./Drivers/Invn/EmbUtils/InvCksum.d \
./Drivers/Invn/EmbUtils/InvFormat.d \
./Drivers/Invn/EmbUtils/InvList.d \
./Drivers/Invn/EmbUtils/InvPrintf.d \
./Drivers/Invn/EmbUtils/InvProtocol.d \
./Drivers/Invn/EmbUtils/InvQueue.d \
./Drivers/Invn/EmbUtils/InvScheduler.d \
./Drivers/Invn/EmbUtils/Logger.d \
./Drivers/Invn/EmbUtils/Message.d \
./Drivers/Invn/EmbUtils/RingByteBuffer.d \
./Drivers/Invn/EmbUtils/UartTxEmulator.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Invn/EmbUtils/%.o Drivers/Invn/EmbUtils/%.su Drivers/Invn/EmbUtils/%.cyclo: ../Drivers/Invn/EmbUtils/%.c Drivers/Invn/EmbUtils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../Core/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Invn-2f-EmbUtils

clean-Drivers-2f-Invn-2f-EmbUtils:
	-$(RM) ./Drivers/Invn/EmbUtils/DataConverter.cyclo ./Drivers/Invn/EmbUtils/DataConverter.d ./Drivers/Invn/EmbUtils/DataConverter.o ./Drivers/Invn/EmbUtils/DataConverter.su ./Drivers/Invn/EmbUtils/ErrorHelper.cyclo ./Drivers/Invn/EmbUtils/ErrorHelper.d ./Drivers/Invn/EmbUtils/ErrorHelper.o ./Drivers/Invn/EmbUtils/ErrorHelper.su ./Drivers/Invn/EmbUtils/InvBasicMath.cyclo ./Drivers/Invn/EmbUtils/InvBasicMath.d ./Drivers/Invn/EmbUtils/InvBasicMath.o ./Drivers/Invn/EmbUtils/InvBasicMath.su ./Drivers/Invn/EmbUtils/InvCksum.cyclo ./Drivers/Invn/EmbUtils/InvCksum.d ./Drivers/Invn/EmbUtils/InvCksum.o ./Drivers/Invn/EmbUtils/InvCksum.su ./Drivers/Invn/EmbUtils/InvFormat.cyclo ./Drivers/Invn/EmbUtils/InvFormat.d ./Drivers/Invn/EmbUtils/InvFormat.o ./Drivers/Invn/EmbUtils/InvFormat.su ./Drivers/Invn/EmbUtils/InvList.cyclo ./Drivers/Invn/EmbUtils/InvList.d ./Drivers/Invn/EmbUtils/InvList.o ./Drivers/Invn/EmbUtils/InvList.su ./Drivers/Invn/EmbUtils/InvPrintf.cyclo ./Drivers/Invn/EmbUtils/InvPrintf.d ./Drivers/Invn/EmbUtils/InvPrintf.o ./Drivers/Invn/EmbUtils/InvPrintf.su ./Drivers/Invn/EmbUtils/InvProtocol.cyclo ./Drivers/Invn/EmbUtils/InvProtocol.d ./Drivers/Invn/EmbUtils/InvProtocol.o ./Drivers/Invn/EmbUtils/InvProtocol.su ./Drivers/Invn/EmbUtils/InvQueue.cyclo ./Drivers/Invn/EmbUtils/InvQueue.d ./Drivers/Invn/EmbUtils/InvQueue.o ./Drivers/Invn/EmbUtils/InvQueue.su ./Drivers/Invn/EmbUtils/InvScheduler.cyclo ./Drivers/Invn/EmbUtils/InvScheduler.d ./Drivers/Invn/EmbUtils/InvScheduler.o ./Drivers/Invn/EmbUtils/InvScheduler.su ./Drivers/Invn/EmbUtils/Logger.cyclo ./Drivers/Invn/EmbUtils/Logger.d ./Drivers/Invn/EmbUtils/Logger.o ./Drivers/Invn/EmbUtils/Logger.su ./Drivers/Invn/EmbUtils/Message.cyclo ./Drivers/Invn/EmbUtils/Message.d ./Drivers/Invn/EmbUtils/Message.o ./Drivers/Invn/EmbUtils/Message.su ./Drivers/Invn/EmbUtils/RingByteBuffer.cyclo ./Drivers/Invn/EmbUtils/RingByteBuffer.d ./Drivers/Invn/EmbUtils/RingByteBuffer.o ./Drivers/Invn/EmbUtils/RingByteBuffer.su ./Drivers/Invn/EmbUtils/UartTxEmulator.cyclo ./Drivers/Invn/EmbUtils/UartTxEmulator.d ./Drivers/Invn/EmbUtils/UartTxEmulator.o ./Drivers/Invn/EmbUtils/UartTxEmulator.su

.PHONY: clean-Drivers-2f-Invn-2f-EmbUtils

