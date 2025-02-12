################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/Profiles/Src/gap_profile.c \
C:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/Profiles/Src/gatt_profile.c 

OBJS += \
./Common/BLE/Modules/Profiles/gap_profile.o \
./Common/BLE/Modules/Profiles/gatt_profile.o 

C_DEPS += \
./Common/BLE/Modules/Profiles/gap_profile.d \
./Common/BLE/Modules/Profiles/gatt_profile.d 


# Each subdirectory must supply rules for building sources it contributes
Common/BLE/Modules/Profiles/gap_profile.o: C:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/Profiles/Src/gap_profile.c Common/BLE/Modules/Profiles/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_WB09KE -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../../Core/Inc -I../../STM32_BLE/App -I../../STM32_BLE/Target -I../../System/Config/Debug_GPIO -I../../System/Interfaces -I../../System/Modules -I../../System/Modules/Flash -I../../System/Modules/NVMDB/Inc -I../../System/Modules/PKAMGR/Inc -I../../System/Modules/Profiles/Inc -I../../System/Modules/RADIO_utils/Inc -I../../System/Modules/RTDebug -I../../System/Startup -I../../Utilities/trace/adv_trace -I../../Drivers/STM32WB0x_HAL_Driver/Inc -I../../Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_BLE -I../../Drivers/CMSIS/Device/ST/STM32WB0X/Include -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I../../Middlewares/ST/STM32_BLE/evt_handler/inc -I../../Middlewares/ST/STM32_BLE/queued_writes/inc -I../../Middlewares/ST/STM32_BLE/stack/include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32WB0x-nucleo -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../../../../Projects/Common/BLE/Interfaces -I../../../../../Projects/Common/BLE/Modules -I../../../../../Projects/Common/BLE/Modules/RTDebug -I../../../../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../../../../Projects/Common/BLE/Modules/Profiles/Inc -I../../../../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../../../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../../../../Projects/Common/BLE/Modules/Flash -I../../../../../Projects/Common/BLE/Startup -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Utilities/trace/adv_trace -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Drivers/STM32WB0x_HAL_Driver/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Interfaces -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/RTDebug -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/RADIO_utils/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/Profiles/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/PKAMGR/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/NVMDB/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/Flash -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Startup -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Utilities/misc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Utilities/sequencer -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Utilities/lpm/tiny_lpm -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Drivers/CMSIS/Device/ST/STM32WB0X/Include -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/evt_handler/inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/queued_writes/inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/stack/include -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Common/BLE/Modules/Profiles/gatt_profile.o: C:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/Profiles/Src/gatt_profile.c Common/BLE/Modules/Profiles/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_WB09KE -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32WB09 -c -I../../Core/Inc -I../../STM32_BLE/App -I../../STM32_BLE/Target -I../../System/Config/Debug_GPIO -I../../System/Interfaces -I../../System/Modules -I../../System/Modules/Flash -I../../System/Modules/NVMDB/Inc -I../../System/Modules/PKAMGR/Inc -I../../System/Modules/Profiles/Inc -I../../System/Modules/RADIO_utils/Inc -I../../System/Modules/RTDebug -I../../System/Startup -I../../Utilities/trace/adv_trace -I../../Drivers/STM32WB0x_HAL_Driver/Inc -I../../Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_BLE -I../../Drivers/CMSIS/Device/ST/STM32WB0X/Include -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -I../../Middlewares/ST/STM32_BLE/evt_handler/inc -I../../Middlewares/ST/STM32_BLE/queued_writes/inc -I../../Middlewares/ST/STM32_BLE/stack/include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32WB0x-nucleo -I../../Middlewares/ST/STM32_BLE/cryptolib/Inc -I../../../../../Projects/Common/BLE/Interfaces -I../../../../../Projects/Common/BLE/Modules -I../../../../../Projects/Common/BLE/Modules/RTDebug -I../../../../../Projects/Common/BLE/Modules/RADIO_utils/Inc -I../../../../../Projects/Common/BLE/Modules/Profiles/Inc -I../../../../../Projects/Common/BLE/Modules/PKAMGR/Inc -I../../../../../Projects/Common/BLE/Modules/NVMDB/Inc -I../../../../../Projects/Common/BLE/Modules/Flash -I../../../../../Projects/Common/BLE/Startup -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Utilities/trace/adv_trace -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Drivers/STM32WB0x_HAL_Driver/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Drivers/STM32WB0x_HAL_Driver/Inc/Legacy -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Interfaces -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/RTDebug -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/RADIO_utils/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/Profiles/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/PKAMGR/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/NVMDB/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Modules/Flash -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Projects/Common/BLE/Startup -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Utilities/misc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Utilities/sequencer -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Utilities/lpm/tiny_lpm -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/Common -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CBC -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/CMAC -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/Common -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/cryptolib/Inc/AES/ECB -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Drivers/CMSIS/Device/ST/STM32WB0X/Include -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/evt_handler/inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/queued_writes/inc -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Middlewares/ST/STM32_BLE/stack/include -IC:/Users/jonac/STM32Cube/Repository/STM32Cube_FW_WB0_V1.1.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Common-2f-BLE-2f-Modules-2f-Profiles

clean-Common-2f-BLE-2f-Modules-2f-Profiles:
	-$(RM) ./Common/BLE/Modules/Profiles/gap_profile.cyclo ./Common/BLE/Modules/Profiles/gap_profile.d ./Common/BLE/Modules/Profiles/gap_profile.o ./Common/BLE/Modules/Profiles/gap_profile.su ./Common/BLE/Modules/Profiles/gatt_profile.cyclo ./Common/BLE/Modules/Profiles/gatt_profile.d ./Common/BLE/Modules/Profiles/gatt_profile.o ./Common/BLE/Modules/Profiles/gatt_profile.su

.PHONY: clean-Common-2f-BLE-2f-Modules-2f-Profiles

