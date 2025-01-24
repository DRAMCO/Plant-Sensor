cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_FULL_LL_DRIVER 
	BLE 
	USE_HAL_DRIVER 
	STM32WBA52xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../System/Interfaces
    ../../System/Modules
    ../../System/Config/Log
    ../../System/Config/LowPower
    ../../System/Config/Debug_GPIO
    ../../System/Config/Flash
    ../../System/Config/ADC_Ctrl
    ../../System/Config/CRC_Ctrl
    ../../STM32_WPAN/App
    ../../STM32_WPAN/Target
    ../../Drivers/STM32WBAxx_HAL_Driver/Inc
    ../../Drivers/STM32WBAxx_HAL_Driver/Inc/Legacy
    ../../Utilities/trace/adv_trace
    ../../Projects/Common/WPAN/Interfaces
    ../../Projects/Common/WPAN/Modules
    ../../Projects/Common/WPAN/Modules/BasicAES
    ../../Projects/Common/WPAN/Modules/Flash
    ../../Projects/Common/WPAN/Modules/MemoryManager
    ../../Projects/Common/WPAN/Modules/Nvm
    ../../Projects/Common/WPAN/Modules/RTDebug
    ../../Projects/Common/WPAN/Modules/SerialCmdInterpreter
    ../../Projects/Common/WPAN/Modules/Log
    ../../Utilities/misc
    ../../Utilities/sequencer
    ../../Utilities/tim_serv
    ../../Utilities/lpm/tiny_lpm
    ../../Middlewares/ST/STM32_WPAN
    ../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/config/ble_full
    ../../Middlewares/ST/STM32_WPAN/ble/svc/Src
    ../../Drivers/CMSIS/Device/ST/STM32WBAxx/Include
    ../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc
    ../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/_40nm_reg_files
    ../../Middlewares/ST/STM32_WPAN/link_layer/ll_cmd_lib/inc/ot_inc
    ../../Middlewares/ST/STM32_WPAN/link_layer/ll_sys/inc
    ../../Middlewares/ST/STM32_WPAN/ble
    ../../Middlewares/ST/STM32_WPAN/ble/stack/include
    ../../Middlewares/ST/STM32_WPAN/ble/stack/include/auto
    ../../Middlewares/ST/STM32_WPAN/ble/svc/Inc
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/app_entry.c
    ../../Core/Src/stm32wbaxx_it.c
    ../../Core/Src/stm32wbaxx_hal_msp.c
    ../../System/Config/Flash/simple_nvm_arbiter_conf.c
    ../../System/Config/CRC_Ctrl/crc_ctrl_conf.c
    ../../System/Config/ADC_Ctrl/adc_ctrl_conf.c
    ../../System/Config/LowPower/user_low_power_config.c
    ../../System/Config/LowPower/peripheral_init.c
    ../../System/Config/Debug_GPIO/app_debug.c
    ../../System/Interfaces/stm32_lpm_if.c
    ../../System/Modules/ble_timer.c
    ../../STM32_WPAN/App/app_ble.c
    ../../STM32_WPAN/Target/bleplat.c
    ../../STM32_WPAN/Target/power_table.c
    ../../STM32_WPAN/Target/linklayer_plat.c
    ../../STM32_WPAN/Target/bpka.c
    ../../STM32_WPAN/Target/ll_sys_if.c
    ../../STM32_WPAN/Target/host_stack_if.c
    ../../Projects/Common/WPAN/Interfaces/adv_trace_usart_if.c
    ../../Projects/Common/WPAN/Interfaces/hw_aes.c
    ../../Projects/Common/WPAN/Interfaces/hw_otp.c
    ../../Projects/Common/WPAN/Interfaces/hw_pka.c
    ../../Projects/Common/WPAN/Interfaces/hw_pka_p256.c
    ../../Projects/Common/WPAN/Interfaces/hw_rng.c
    ../../Projects/Common/WPAN/Interfaces/timer_if.c
    ../../Projects/Common/WPAN/Modules/adc_ctrl.c
    ../../Projects/Common/WPAN/Modules/app_sys.c
    ../../Projects/Common/WPAN/Modules/crc_ctrl.c
    ../../Projects/Common/WPAN/Modules/otp.c
    ../../Projects/Common/WPAN/Modules/scm.c
    ../../Projects/Common/WPAN/Modules/stm_list.c
    ../../Projects/Common/WPAN/Modules/stm_queue.c
    ../../Projects/Common/WPAN/Modules/temp_measurement.c
    ../../Projects/Common/WPAN/Modules/BasicAES/baes_cmac.c
    ../../Projects/Common/WPAN/Modules/BasicAES/baes_ccm.c
    ../../Projects/Common/WPAN/Modules/BasicAES/baes_ecb.c
    ../../Projects/Common/WPAN/Modules/Flash/flash_driver.c
    ../../Projects/Common/WPAN/Modules/Flash/flash_manager.c
    ../../Projects/Common/WPAN/Modules/Flash/rf_timing_synchro.c
    ../../Projects/Common/WPAN/Modules/Flash/simple_nvm_arbiter.c
    ../../Projects/Common/WPAN/Modules/MemoryManager/advanced_memory_manager.c
    ../../Projects/Common/WPAN/Modules/MemoryManager/stm32_mm.c
    ../../Projects/Common/WPAN/Modules/Nvm/nvm_emul.c
    ../../Projects/Common/WPAN/Modules/RTDebug/RTDebug.c
    ../../Projects/Common/WPAN/Modules/RTDebug/RTDebug_dtb.c
    ../../Projects/Common/WPAN/Modules/SerialCmdInterpreter/serial_cmd_interpreter.c
    ../../Projects/Common/WPAN/Modules/Log/log_module.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_ll_utils.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_ll_exti.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_adc.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_adc_ex.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_dma.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_dma_ex.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_rcc.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_rcc_ex.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_cortex.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_flash.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_flash_ex.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_gpio.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_exti.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_pwr.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_pwr_ex.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_crc.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_crc_ex.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_ll_icache.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_ll_gpio.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_ramcfg.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_rng.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_rng_ex.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_rtc.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_rtc_ex.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_ll_tim.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_ll_dma.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_uart.c
    ../../Drivers/STM32WBAxx_HAL_Driver/Src/stm32wbaxx_hal_uart_ex.c
    ../../Core/Src/system_stm32wbaxx.c
    ../../Utilities/trace/adv_trace/stm32_adv_trace.c
    ../../Utilities/misc/stm32_mem.c
    ../../Utilities/misc/stm32_systime.c
    ../../Utilities/misc/stm32_tiny_sscanf.c
    ../../Utilities/misc/stm32_tiny_vsnprintf.c
    ../../Utilities/sequencer/stm32_seq.c
    ../../Utilities/tim_serv/stm32_timer.c
    ../../Utilities/lpm/tiny_lpm/stm32_lpm.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32wba52xx.s
    ../../Projects/Common/WPAN/Startup/stm32wbaxx_ResetHandler_GCC.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


