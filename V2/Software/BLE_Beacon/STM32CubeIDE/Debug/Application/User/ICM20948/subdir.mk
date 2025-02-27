################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Core/Src/ICM20948.c 

OBJS += \
./Application/User/ICM20948/ICM20948.o 

C_DEPS += \
./Application/User/ICM20948/ICM20948.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/ICM20948/ICM20948.o: C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/BLE_Beacon/Core/Src/ICM20948.c Application/User/ICM20948/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_WB09KE -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../../Core/Inc -I../../STM32_BLE/App -I../../Core/Config -I../../STM32_BLE/Target -I../../System/Config/Debug_GPIO -I../../System/Interfaces -I../../System/Modules -I../../System/Modules/Flash -I../../System/Modules/NVMDB/Inc -I../../System/Modules/PKAMGR/Inc -I../../System/Modules/Profiles/Inc -I../../System/Modules/RADIO_utils/Inc -I../../System/Modules/RTDebug -I../../System/Startup -I../../Utilities/trace/adv_trace -I../../Drivers/STM32WB0x_HAL_Driver/Inc -I../../Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_BLE -I../../Drivers/CMSIS/Device/ST/STM32WB0X/Include -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I../../Middlewares/ST/STM32_BLE/evt_handler/inc -I../../Middlewares/ST/STM32_BLE/queued_writes/inc -I../../Middlewares/ST/STM32_BLE/stack/include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32WB0x-nucleo -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../../../../Projects/Common/BLE/Interfaces -I../../../../../Projects/Common/BLE/Modules -I../../../../../Projects/Common/BLE/Modules/RTDebug -I../../../../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../../../../Projects/Common/BLE/Modules/Profiles/Inc -I../../../../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../../../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../../../../Projects/Common/BLE/Modules/Flash -I../../../../../Projects/Common/BLE/Startup -I../../Projects/Common/BLE/Interfaces -I../../Projects/Common/BLE/Modules -I../../Projects/Common/BLE/Modules/RTDebug -I../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../Projects/Common/BLE/Modules/Profiles/Inc -I../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../Projects/Common/BLE/Modules/Flash -I../../Projects/Common/BLE/Startup -O0 -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Application-2f-User-2f-ICM20948

clean-Application-2f-User-2f-ICM20948:
	-$(RM) ./Application/User/ICM20948/ICM20948.cyclo ./Application/User/ICM20948/ICM20948.d ./Application/User/ICM20948/ICM20948.o ./Application/User/ICM20948/ICM20948.su

.PHONY: clean-Application-2f-User-2f-ICM20948

