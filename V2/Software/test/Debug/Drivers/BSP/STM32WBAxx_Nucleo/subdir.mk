################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32WBAxx_Nucleo/stm32wbaxx_nucleo.c 

OBJS += \
./Drivers/BSP/STM32WBAxx_Nucleo/stm32wbaxx_nucleo.o 

C_DEPS += \
./Drivers/BSP/STM32WBAxx_Nucleo/stm32wbaxx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32WBAxx_Nucleo/%.o Drivers/BSP/STM32WBAxx_Nucleo/%.su Drivers/BSP/STM32WBAxx_Nucleo/%.cyclo: ../Drivers/BSP/STM32WBAxx_Nucleo/%.c Drivers/BSP/STM32WBAxx_Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DBLE -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../Core/Inc -I../System/Interfaces -I../System/Modules -I../System/Modules/BasicAES -I../System/Modules/Flash -I../System/Modules/MemoryManager -I../System/Modules/Nvm -I../System/Modules/RTDebug -I../System/Modules/SerialCmdInterpreter -I../System/Config/Log -I../System/Config/LowPower -I../System/Config/Debug_GPIO -I../System/Config/Flash -I../System/Config/ADC_Ctrl -I../System/Config/CRC_Ctrl -I../STM32_WPAN/App -I../STM32_WPAN/Target -I../Drivers/STM32WBAxx_HAL_Driver/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/tim_serv -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/config/ble_basic -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Drivers/BSP/STM32WBAxx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc -I../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/_40nm_reg_files -I../Middlewares/ST/STM32_WPAN/link_layer/ll_sys/inc -I../Middlewares/ST/STM32_WPAN/ble -I../Middlewares/ST/STM32_WPAN/ble/stack/include -I../Middlewares/ST/STM32_WPAN/ble/stack/include/auto -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32WBAxx_Nucleo

clean-Drivers-2f-BSP-2f-STM32WBAxx_Nucleo:
	-$(RM) ./Drivers/BSP/STM32WBAxx_Nucleo/stm32wbaxx_nucleo.cyclo ./Drivers/BSP/STM32WBAxx_Nucleo/stm32wbaxx_nucleo.d ./Drivers/BSP/STM32WBAxx_Nucleo/stm32wbaxx_nucleo.o ./Drivers/BSP/STM32WBAxx_Nucleo/stm32wbaxx_nucleo.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32WBAxx_Nucleo

