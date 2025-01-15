################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Invn/Devices/DeviceIcm20948.c \
../Drivers/Invn/Devices/HostSerif.c \
../Drivers/Invn/Devices/Sensor.c \
../Drivers/Invn/Devices/VSensorId.c 

OBJS += \
./Drivers/Invn/Devices/DeviceIcm20948.o \
./Drivers/Invn/Devices/HostSerif.o \
./Drivers/Invn/Devices/Sensor.o \
./Drivers/Invn/Devices/VSensorId.o 

C_DEPS += \
./Drivers/Invn/Devices/DeviceIcm20948.d \
./Drivers/Invn/Devices/HostSerif.d \
./Drivers/Invn/Devices/Sensor.d \
./Drivers/Invn/Devices/VSensorId.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Invn/Devices/%.o Drivers/Invn/Devices/%.su Drivers/Invn/Devices/%.cyclo: ../Drivers/Invn/Devices/%.c Drivers/Invn/Devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../Core/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Invn-2f-Devices

clean-Drivers-2f-Invn-2f-Devices:
	-$(RM) ./Drivers/Invn/Devices/DeviceIcm20948.cyclo ./Drivers/Invn/Devices/DeviceIcm20948.d ./Drivers/Invn/Devices/DeviceIcm20948.o ./Drivers/Invn/Devices/DeviceIcm20948.su ./Drivers/Invn/Devices/HostSerif.cyclo ./Drivers/Invn/Devices/HostSerif.d ./Drivers/Invn/Devices/HostSerif.o ./Drivers/Invn/Devices/HostSerif.su ./Drivers/Invn/Devices/Sensor.cyclo ./Drivers/Invn/Devices/Sensor.d ./Drivers/Invn/Devices/Sensor.o ./Drivers/Invn/Devices/Sensor.su ./Drivers/Invn/Devices/VSensorId.cyclo ./Drivers/Invn/Devices/VSensorId.d ./Drivers/Invn/Devices/VSensorId.o ./Drivers/Invn/Devices/VSensorId.su

.PHONY: clean-Drivers-2f-Invn-2f-Devices

