################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Utilities/subdir.mk
-include Middlewares/STM32_BLE/subdir.mk
-include Invn/VSensor/subdir.mk
-include Invn/EmbUtils/subdir.mk
-include Invn/DynamicProtocol/subdir.mk
-include Invn/Devices/Drivers/Icm20948/subdir.mk
-include Invn/Devices/subdir.mk
-include Drivers/STM32WB0x_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/NUCLEO-WB09KE/subdir.mk
-include Common/BLE/Startup/subdir.mk
-include Common/BLE/Modules/RTDebug/subdir.mk
-include Common/BLE/Modules/RADIO_utils/subdir.mk
-include Common/BLE/Modules/Profiles/subdir.mk
-include Common/BLE/Modules/PKAMGR/subdir.mk
-include Common/BLE/Modules/NVMDB/subdir.mk
-include Common/BLE/Modules/Flash/subdir.mk
-include Common/BLE/Modules/subdir.mk
-include Common/BLE/Interface/subdir.mk
-include Application/User/System/Interfaces/subdir.mk
-include Application/User/System/Config/Debug_GPIO/subdir.mk
-include Application/User/Startup/subdir.mk
-include Application/User/STM32_BLE/Target/subdir.mk
-include Application/User/STM32_BLE/App/subdir.mk
-include Application/User/RTT/subdir.mk
-include Application/User/ICM20948/subdir.mk
-include Application/User/Core/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := BLE_Beacon
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BLE_Beacon.elf \

MAP_FILES += \
BLE_Beacon.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BLE_Beacon.list \

OBJCOPY_HEX += \
BLE_Beacon.hex \

OBJCOPY_BIN += \
BLE_Beacon.bin \


# All Target
all: main-build

# Main-build Target
main-build: BLE_Beacon.elf secondary-outputs

# Tool invocations
BLE_Beacon.elf BLE_Beacon.map: $(OBJS) $(USER_OBJS) C:\Users\jonac\Documents\GitHub\Plant-Sensor\V2\Software\BLE_Beacon\STM32CubeIDE\STM32WB09KEVX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "BLE_Beacon.elf" @"objects.list" $(USER_OBJS) -Wl,--start-group $(LIBS) -Wl,--end-group -mcpu=cortex-m0plus -T"C:\Users\jonac\Documents\GitHub\Plant-Sensor\V2\Software\BLE_Beacon\STM32CubeIDE\STM32WB09KEVX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BLE_Beacon.map" -Wl,--gc-sections -static -L../../Middlewares/ST/STM32_BLE/cryptolib -L../../Middlewares/ST/STM32_BLE/stack/lib -z noexecstack --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BLE_Beacon.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BLE_Beacon.list"
	@echo 'Finished building: $@'
	@echo ' '

BLE_Beacon.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "BLE_Beacon.hex"
	@echo 'Finished building: $@'
	@echo ' '

BLE_Beacon.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "BLE_Beacon.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) BLE_Beacon.bin BLE_Beacon.elf BLE_Beacon.hex BLE_Beacon.list BLE_Beacon.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
