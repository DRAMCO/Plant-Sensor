################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/test2/Middlewares/ST/STM32_WPAN/link_layer/ll_sys/src/ll_sys_cs.c \
C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/test2/Middlewares/ST/STM32_WPAN/link_layer/ll_sys/src/ll_sys_dp_slp.c \
C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/test2/Middlewares/ST/STM32_WPAN/link_layer/ll_sys/src/ll_sys_intf.c \
C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/test2/Middlewares/ST/STM32_WPAN/link_layer/ll_sys/src/ll_sys_startup.c 

OBJS += \
./Middlewares/STM32_WPAN/LinkLayer/ll_sys_cs.o \
./Middlewares/STM32_WPAN/LinkLayer/ll_sys_dp_slp.o \
./Middlewares/STM32_WPAN/LinkLayer/ll_sys_intf.o \
./Middlewares/STM32_WPAN/LinkLayer/ll_sys_startup.o 

C_DEPS += \
./Middlewares/STM32_WPAN/LinkLayer/ll_sys_cs.d \
./Middlewares/STM32_WPAN/LinkLayer/ll_sys_dp_slp.d \
./Middlewares/STM32_WPAN/LinkLayer/ll_sys_intf.d \
./Middlewares/STM32_WPAN/LinkLayer/ll_sys_startup.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/STM32_WPAN/LinkLayer/ll_sys_cs.o: C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/test2/Middlewares/ST/STM32_WPAN/link_layer/ll_sys/src/ll_sys_cs.c Middlewares/STM32_WPAN/LinkLayer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DBLE -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../../Core/Inc -I../../System/Interfaces -I../../System/Modules -I../../System/Config/Log -I../../System/Config/LowPower -I../../System/Config/Debug_GPIO -I../../System/Config/Flash -I../../System/Config/ADC_Ctrl -I../../System/Config/CRC_Ctrl -I../../STM32_WPAN/App -I../../STM32_WPAN/Target -I../../Drivers/STM32WBAxx_HAL_Driver/Inc -I../../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../../Utilities/trace/adv_trace -I../../Projects/Common/WPAN/Interfaces -I../../Projects/Common/WPAN/Modules -I../../Projects/Common/WPAN/Modules/BasicAES -I../../Projects/Common/WPAN/Modules/Flash -I../../Projects/Common/WPAN/Modules/MemoryManager -I../../Projects/Common/WPAN/Modules/Nvm -I../../Projects/Common/WPAN/Modules/RTDebug -I../../Projects/Common/WPAN/Modules/SerialCmdInterpreter -I../../Projects/Common/WPAN/Modules/Log -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/tim_serv -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_WPAN -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/config/ble_full -I../../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/_40nm_reg_files -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/ot_inc -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_sys/inc -I../../Middlewares/ST/STM32_WPAN/ble -I../../Middlewares/ST/STM32_WPAN/ble/stack/include -I../../Middlewares/ST/STM32_WPAN/ble/stack/include/auto -I../../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_WPAN/LinkLayer/ll_sys_dp_slp.o: C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/test2/Middlewares/ST/STM32_WPAN/link_layer/ll_sys/src/ll_sys_dp_slp.c Middlewares/STM32_WPAN/LinkLayer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DBLE -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../../Core/Inc -I../../System/Interfaces -I../../System/Modules -I../../System/Config/Log -I../../System/Config/LowPower -I../../System/Config/Debug_GPIO -I../../System/Config/Flash -I../../System/Config/ADC_Ctrl -I../../System/Config/CRC_Ctrl -I../../STM32_WPAN/App -I../../STM32_WPAN/Target -I../../Drivers/STM32WBAxx_HAL_Driver/Inc -I../../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../../Utilities/trace/adv_trace -I../../Projects/Common/WPAN/Interfaces -I../../Projects/Common/WPAN/Modules -I../../Projects/Common/WPAN/Modules/BasicAES -I../../Projects/Common/WPAN/Modules/Flash -I../../Projects/Common/WPAN/Modules/MemoryManager -I../../Projects/Common/WPAN/Modules/Nvm -I../../Projects/Common/WPAN/Modules/RTDebug -I../../Projects/Common/WPAN/Modules/SerialCmdInterpreter -I../../Projects/Common/WPAN/Modules/Log -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/tim_serv -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_WPAN -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/config/ble_full -I../../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/_40nm_reg_files -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/ot_inc -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_sys/inc -I../../Middlewares/ST/STM32_WPAN/ble -I../../Middlewares/ST/STM32_WPAN/ble/stack/include -I../../Middlewares/ST/STM32_WPAN/ble/stack/include/auto -I../../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_WPAN/LinkLayer/ll_sys_intf.o: C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/test2/Middlewares/ST/STM32_WPAN/link_layer/ll_sys/src/ll_sys_intf.c Middlewares/STM32_WPAN/LinkLayer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DBLE -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../../Core/Inc -I../../System/Interfaces -I../../System/Modules -I../../System/Config/Log -I../../System/Config/LowPower -I../../System/Config/Debug_GPIO -I../../System/Config/Flash -I../../System/Config/ADC_Ctrl -I../../System/Config/CRC_Ctrl -I../../STM32_WPAN/App -I../../STM32_WPAN/Target -I../../Drivers/STM32WBAxx_HAL_Driver/Inc -I../../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../../Utilities/trace/adv_trace -I../../Projects/Common/WPAN/Interfaces -I../../Projects/Common/WPAN/Modules -I../../Projects/Common/WPAN/Modules/BasicAES -I../../Projects/Common/WPAN/Modules/Flash -I../../Projects/Common/WPAN/Modules/MemoryManager -I../../Projects/Common/WPAN/Modules/Nvm -I../../Projects/Common/WPAN/Modules/RTDebug -I../../Projects/Common/WPAN/Modules/SerialCmdInterpreter -I../../Projects/Common/WPAN/Modules/Log -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/tim_serv -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_WPAN -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/config/ble_full -I../../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/_40nm_reg_files -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/ot_inc -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_sys/inc -I../../Middlewares/ST/STM32_WPAN/ble -I../../Middlewares/ST/STM32_WPAN/ble/stack/include -I../../Middlewares/ST/STM32_WPAN/ble/stack/include/auto -I../../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_WPAN/LinkLayer/ll_sys_startup.o: C:/Users/jonac/Documents/GitHub/Plant-Sensor/V2/Software/test2/Middlewares/ST/STM32_WPAN/link_layer/ll_sys/src/ll_sys_startup.c Middlewares/STM32_WPAN/LinkLayer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DBLE -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../../Core/Inc -I../../System/Interfaces -I../../System/Modules -I../../System/Config/Log -I../../System/Config/LowPower -I../../System/Config/Debug_GPIO -I../../System/Config/Flash -I../../System/Config/ADC_Ctrl -I../../System/Config/CRC_Ctrl -I../../STM32_WPAN/App -I../../STM32_WPAN/Target -I../../Drivers/STM32WBAxx_HAL_Driver/Inc -I../../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../../Utilities/trace/adv_trace -I../../Projects/Common/WPAN/Interfaces -I../../Projects/Common/WPAN/Modules -I../../Projects/Common/WPAN/Modules/BasicAES -I../../Projects/Common/WPAN/Modules/Flash -I../../Projects/Common/WPAN/Modules/MemoryManager -I../../Projects/Common/WPAN/Modules/Nvm -I../../Projects/Common/WPAN/Modules/RTDebug -I../../Projects/Common/WPAN/Modules/SerialCmdInterpreter -I../../Projects/Common/WPAN/Modules/Log -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/tim_serv -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_WPAN -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/config/ble_full -I../../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/_40nm_reg_files -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/ot_inc -I../../Middlewares/ST/STM32_WPAN/link_layer/ll_sys/inc -I../../Middlewares/ST/STM32_WPAN/ble -I../../Middlewares/ST/STM32_WPAN/ble/stack/include -I../../Middlewares/ST/STM32_WPAN/ble/stack/include/auto -I../../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-STM32_WPAN-2f-LinkLayer

clean-Middlewares-2f-STM32_WPAN-2f-LinkLayer:
	-$(RM) ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_cs.cyclo ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_cs.d ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_cs.o ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_cs.su ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_dp_slp.cyclo ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_dp_slp.d ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_dp_slp.o ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_dp_slp.su ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_intf.cyclo ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_intf.d ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_intf.o ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_intf.su ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_startup.cyclo ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_startup.d ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_startup.o ./Middlewares/STM32_WPAN/LinkLayer/ll_sys_startup.su

.PHONY: clean-Middlewares-2f-STM32_WPAN-2f-LinkLayer

