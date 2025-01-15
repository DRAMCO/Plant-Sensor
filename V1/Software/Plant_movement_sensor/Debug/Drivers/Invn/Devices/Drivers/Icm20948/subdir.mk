################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Augmented.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Setup.c \
../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Transport.c 

OBJS += \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Augmented.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Setup.o \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Transport.o 

C_DEPS += \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Augmented.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Setup.d \
./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Transport.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Invn/Devices/Drivers/Icm20948/%.o Drivers/Invn/Devices/Drivers/Icm20948/%.su Drivers/Invn/Devices/Drivers/Icm20948/%.cyclo: ../Drivers/Invn/Devices/Drivers/Icm20948/%.c Drivers/Invn/Devices/Drivers/Icm20948/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../Core/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Invn-2f-Devices-2f-Drivers-2f-Icm20948

clean-Drivers-2f-Invn-2f-Devices-2f-Drivers-2f-Icm20948:
	-$(RM) ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Augmented.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Augmented.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Augmented.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Augmented.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Setup.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Setup.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Setup.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Setup.su ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Transport.cyclo ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Transport.d ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Transport.o ./Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Transport.su

.PHONY: clean-Drivers-2f-Invn-2f-Devices-2f-Drivers-2f-Icm20948

