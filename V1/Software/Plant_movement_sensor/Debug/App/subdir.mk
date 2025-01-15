################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/app_MadgwickAHRS.c \
../App/app_MahonyAHRS.c \
../App/app_adc.c \
../App/app_gateway.c \
../App/app_gnss.c \
../App/app_hal_pps.c \
../App/app_hal_sync.c \
../App/app_imu.c \
../App/app_init.c \
../App/app_led.c \
../App/app_lorawan_aes.c \
../App/app_lorawan_cmac.c \
../App/app_lorawan_crypto.c \
../App/app_lorawan_utilities.c \
../App/app_network_connect.c \
../App/app_node_gnsspps.c \
../App/app_radio_available.c \
../App/app_rtc.c \
../App/app_rtc_sync.c \
../App/app_supercap.c \
../App/common.c 

OBJS += \
./App/app_MadgwickAHRS.o \
./App/app_MahonyAHRS.o \
./App/app_adc.o \
./App/app_gateway.o \
./App/app_gnss.o \
./App/app_hal_pps.o \
./App/app_hal_sync.o \
./App/app_imu.o \
./App/app_init.o \
./App/app_led.o \
./App/app_lorawan_aes.o \
./App/app_lorawan_cmac.o \
./App/app_lorawan_crypto.o \
./App/app_lorawan_utilities.o \
./App/app_network_connect.o \
./App/app_node_gnsspps.o \
./App/app_radio_available.o \
./App/app_rtc.o \
./App/app_rtc_sync.o \
./App/app_supercap.o \
./App/common.o 

C_DEPS += \
./App/app_MadgwickAHRS.d \
./App/app_MahonyAHRS.d \
./App/app_adc.d \
./App/app_gateway.d \
./App/app_gnss.d \
./App/app_hal_pps.d \
./App/app_hal_sync.d \
./App/app_imu.d \
./App/app_init.d \
./App/app_led.d \
./App/app_lorawan_aes.d \
./App/app_lorawan_cmac.d \
./App/app_lorawan_crypto.d \
./App/app_lorawan_utilities.d \
./App/app_network_connect.d \
./App/app_node_gnsspps.d \
./App/app_radio_available.d \
./App/app_rtc.d \
./App/app_rtc_sync.d \
./App/app_supercap.d \
./App/common.d 


# Each subdirectory must supply rules for building sources it contributes
App/%.o App/%.su App/%.cyclo: ../App/%.c App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WBA52xx -c -I../Core/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc -I../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBAxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App

clean-App:
	-$(RM) ./App/app_MadgwickAHRS.cyclo ./App/app_MadgwickAHRS.d ./App/app_MadgwickAHRS.o ./App/app_MadgwickAHRS.su ./App/app_MahonyAHRS.cyclo ./App/app_MahonyAHRS.d ./App/app_MahonyAHRS.o ./App/app_MahonyAHRS.su ./App/app_adc.cyclo ./App/app_adc.d ./App/app_adc.o ./App/app_adc.su ./App/app_gateway.cyclo ./App/app_gateway.d ./App/app_gateway.o ./App/app_gateway.su ./App/app_gnss.cyclo ./App/app_gnss.d ./App/app_gnss.o ./App/app_gnss.su ./App/app_hal_pps.cyclo ./App/app_hal_pps.d ./App/app_hal_pps.o ./App/app_hal_pps.su ./App/app_hal_sync.cyclo ./App/app_hal_sync.d ./App/app_hal_sync.o ./App/app_hal_sync.su ./App/app_imu.cyclo ./App/app_imu.d ./App/app_imu.o ./App/app_imu.su ./App/app_init.cyclo ./App/app_init.d ./App/app_init.o ./App/app_init.su ./App/app_led.cyclo ./App/app_led.d ./App/app_led.o ./App/app_led.su ./App/app_lorawan_aes.cyclo ./App/app_lorawan_aes.d ./App/app_lorawan_aes.o ./App/app_lorawan_aes.su ./App/app_lorawan_cmac.cyclo ./App/app_lorawan_cmac.d ./App/app_lorawan_cmac.o ./App/app_lorawan_cmac.su ./App/app_lorawan_crypto.cyclo ./App/app_lorawan_crypto.d ./App/app_lorawan_crypto.o ./App/app_lorawan_crypto.su ./App/app_lorawan_utilities.cyclo ./App/app_lorawan_utilities.d ./App/app_lorawan_utilities.o ./App/app_lorawan_utilities.su ./App/app_network_connect.cyclo ./App/app_network_connect.d ./App/app_network_connect.o ./App/app_network_connect.su ./App/app_node_gnsspps.cyclo ./App/app_node_gnsspps.d ./App/app_node_gnsspps.o ./App/app_node_gnsspps.su ./App/app_radio_available.cyclo ./App/app_radio_available.d ./App/app_radio_available.o ./App/app_radio_available.su ./App/app_rtc.cyclo ./App/app_rtc.d ./App/app_rtc.o ./App/app_rtc.su ./App/app_rtc_sync.cyclo ./App/app_rtc_sync.d ./App/app_rtc_sync.o ./App/app_rtc_sync.su ./App/app_supercap.cyclo ./App/app_supercap.d ./App/app_supercap.o ./App/app_supercap.su ./App/common.cyclo ./App/common.d ./App/common.o ./App/common.su

.PHONY: clean-App

