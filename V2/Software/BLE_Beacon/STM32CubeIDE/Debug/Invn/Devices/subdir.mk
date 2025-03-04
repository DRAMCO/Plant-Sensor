################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices/DeviceIcm20948.c \
C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices/HostSerif.c \
C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices/Sensor.c \
C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices/VSensorId.c 

OBJS += \
./Invn/Devices/DeviceIcm20948.o \
./Invn/Devices/HostSerif.o \
./Invn/Devices/Sensor.o \
./Invn/Devices/VSensorId.o 

C_DEPS += \
./Invn/Devices/DeviceIcm20948.d \
./Invn/Devices/HostSerif.d \
./Invn/Devices/Sensor.d \
./Invn/Devices/VSensorId.d 


# Each subdirectory must supply rules for building sources it contributes
Invn/Devices/DeviceIcm20948.o: C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices/DeviceIcm20948.c Invn/Devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_WB09KE -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../../Core/Inc -I"C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices" -I../../STM32_BLE/App -I../../Core/Config -I../../STM32_BLE/Target -I../../System/Config/Debug_GPIO -I../../System/Interfaces -I../../System/Modules -I../../System/Modules/Flash -I../../System/Modules/NVMDB/Inc -I../../System/Modules/PKAMGR/Inc -I../../System/Modules/Profiles/Inc -I../../System/Modules/RADIO_utils/Inc -I../../System/Modules/RTDebug -I../../System/Startup -I../../Utilities/trace/adv_trace -I../../Drivers/STM32WB0x_HAL_Driver/Inc -I../../Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_BLE -I../../Drivers/CMSIS/Device/ST/STM32WB0X/Include -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I../../Middlewares/ST/STM32_BLE/evt_handler/inc -I../../Middlewares/ST/STM32_BLE/queued_writes/inc -I../../Middlewares/ST/STM32_BLE/stack/include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32WB0x-nucleo -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../../../../Projects/Common/BLE/Interfaces -I../../../../../Projects/Common/BLE/Modules -I../../../../../Projects/Common/BLE/Modules/RTDebug -I../../../../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../../../../Projects/Common/BLE/Modules/Profiles/Inc -I../../../../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../../../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../../../../Projects/Common/BLE/Modules/Flash -I../../../../../Projects/Common/BLE/Startup -I../../Projects/Common/BLE/Interfaces -I../../Projects/Common/BLE/Modules -I../../Projects/Common/BLE/Modules/RTDebug -I../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../Projects/Common/BLE/Modules/Profiles/Inc -I../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../Projects/Common/BLE/Modules/Flash -I../../Projects/Common/BLE/Startup -O0 -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Invn/Devices/HostSerif.o: C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices/HostSerif.c Invn/Devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_WB09KE -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../../Core/Inc -I"C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices" -I../../STM32_BLE/App -I../../Core/Config -I../../STM32_BLE/Target -I../../System/Config/Debug_GPIO -I../../System/Interfaces -I../../System/Modules -I../../System/Modules/Flash -I../../System/Modules/NVMDB/Inc -I../../System/Modules/PKAMGR/Inc -I../../System/Modules/Profiles/Inc -I../../System/Modules/RADIO_utils/Inc -I../../System/Modules/RTDebug -I../../System/Startup -I../../Utilities/trace/adv_trace -I../../Drivers/STM32WB0x_HAL_Driver/Inc -I../../Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_BLE -I../../Drivers/CMSIS/Device/ST/STM32WB0X/Include -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I../../Middlewares/ST/STM32_BLE/evt_handler/inc -I../../Middlewares/ST/STM32_BLE/queued_writes/inc -I../../Middlewares/ST/STM32_BLE/stack/include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32WB0x-nucleo -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../../../../Projects/Common/BLE/Interfaces -I../../../../../Projects/Common/BLE/Modules -I../../../../../Projects/Common/BLE/Modules/RTDebug -I../../../../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../../../../Projects/Common/BLE/Modules/Profiles/Inc -I../../../../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../../../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../../../../Projects/Common/BLE/Modules/Flash -I../../../../../Projects/Common/BLE/Startup -I../../Projects/Common/BLE/Interfaces -I../../Projects/Common/BLE/Modules -I../../Projects/Common/BLE/Modules/RTDebug -I../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../Projects/Common/BLE/Modules/Profiles/Inc -I../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../Projects/Common/BLE/Modules/Flash -I../../Projects/Common/BLE/Startup -O0 -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Invn/Devices/Sensor.o: C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices/Sensor.c Invn/Devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_WB09KE -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../../Core/Inc -I"C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices" -I../../STM32_BLE/App -I../../Core/Config -I../../STM32_BLE/Target -I../../System/Config/Debug_GPIO -I../../System/Interfaces -I../../System/Modules -I../../System/Modules/Flash -I../../System/Modules/NVMDB/Inc -I../../System/Modules/PKAMGR/Inc -I../../System/Modules/Profiles/Inc -I../../System/Modules/RADIO_utils/Inc -I../../System/Modules/RTDebug -I../../System/Startup -I../../Utilities/trace/adv_trace -I../../Drivers/STM32WB0x_HAL_Driver/Inc -I../../Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_BLE -I../../Drivers/CMSIS/Device/ST/STM32WB0X/Include -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I../../Middlewares/ST/STM32_BLE/evt_handler/inc -I../../Middlewares/ST/STM32_BLE/queued_writes/inc -I../../Middlewares/ST/STM32_BLE/stack/include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32WB0x-nucleo -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../../../../Projects/Common/BLE/Interfaces -I../../../../../Projects/Common/BLE/Modules -I../../../../../Projects/Common/BLE/Modules/RTDebug -I../../../../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../../../../Projects/Common/BLE/Modules/Profiles/Inc -I../../../../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../../../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../../../../Projects/Common/BLE/Modules/Flash -I../../../../../Projects/Common/BLE/Startup -I../../Projects/Common/BLE/Interfaces -I../../Projects/Common/BLE/Modules -I../../Projects/Common/BLE/Modules/RTDebug -I../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../Projects/Common/BLE/Modules/Profiles/Inc -I../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../Projects/Common/BLE/Modules/Flash -I../../Projects/Common/BLE/Startup -O0 -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Invn/Devices/VSensorId.o: C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices/VSensorId.c Invn/Devices/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_WB09KE -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../../Core/Inc -I"C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Invn/Devices" -I../../STM32_BLE/App -I../../Core/Config -I../../STM32_BLE/Target -I../../System/Config/Debug_GPIO -I../../System/Interfaces -I../../System/Modules -I../../System/Modules/Flash -I../../System/Modules/NVMDB/Inc -I../../System/Modules/PKAMGR/Inc -I../../System/Modules/Profiles/Inc -I../../System/Modules/RADIO_utils/Inc -I../../System/Modules/RTDebug -I../../System/Startup -I../../Utilities/trace/adv_trace -I../../Drivers/STM32WB0x_HAL_Driver/Inc -I../../Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_BLE -I../../Drivers/CMSIS/Device/ST/STM32WB0X/Include -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I../../Middlewares/ST/STM32_BLE/evt_handler/inc -I../../Middlewares/ST/STM32_BLE/queued_writes/inc -I../../Middlewares/ST/STM32_BLE/stack/include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32WB0x-nucleo -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../../../../Projects/Common/BLE/Interfaces -I../../../../../Projects/Common/BLE/Modules -I../../../../../Projects/Common/BLE/Modules/RTDebug -I../../../../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../../../../Projects/Common/BLE/Modules/Profiles/Inc -I../../../../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../../../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../../../../Projects/Common/BLE/Modules/Flash -I../../../../../Projects/Common/BLE/Startup -I../../Projects/Common/BLE/Interfaces -I../../Projects/Common/BLE/Modules -I../../Projects/Common/BLE/Modules/RTDebug -I../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../Projects/Common/BLE/Modules/Profiles/Inc -I../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../Projects/Common/BLE/Modules/Flash -I../../Projects/Common/BLE/Startup -O0 -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Invn-2f-Devices

clean-Invn-2f-Devices:
	-$(RM) ./Invn/Devices/DeviceIcm20948.cyclo ./Invn/Devices/DeviceIcm20948.d ./Invn/Devices/DeviceIcm20948.o ./Invn/Devices/DeviceIcm20948.su ./Invn/Devices/HostSerif.cyclo ./Invn/Devices/HostSerif.d ./Invn/Devices/HostSerif.o ./Invn/Devices/HostSerif.su ./Invn/Devices/Sensor.cyclo ./Invn/Devices/Sensor.d ./Invn/Devices/Sensor.o ./Invn/Devices/Sensor.su ./Invn/Devices/VSensorId.cyclo ./Invn/Devices/VSensorId.d ./Invn/Devices/VSensorId.o ./Invn/Devices/VSensorId.su

.PHONY: clean-Invn-2f-Devices

