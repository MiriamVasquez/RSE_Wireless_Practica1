################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utilities/subdir.mk
-include startup/subdir.mk
-include source/subdir.mk
-include ieee_802.15.4/phy/source/MKW41Z/subdir.mk
-include ieee_802.15.4/mac/source/App/subdir.mk
-include freertos/subdir.mk
-include framework/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/subdir.mk
-include framework/XCVR/MKW41Z4/subdir.mk
-include framework/TimersManager/Source/subdir.mk
-include framework/SerialManager/Source/UART_Adapter/subdir.mk
-include framework/SerialManager/Source/SPI_Adapter/subdir.mk
-include framework/SerialManager/Source/I2C_Adapter/subdir.mk
-include framework/SerialManager/Source/subdir.mk
-include framework/SecLib/subdir.mk
-include framework/Reset/subdir.mk
-include framework/RNG/Source/subdir.mk
-include framework/Panic/Source/subdir.mk
-include framework/OSAbstraction/Source/subdir.mk
-include framework/ModuleInfo/subdir.mk
-include framework/Messaging/Source/subdir.mk
-include framework/MemManager/Source/subdir.mk
-include framework/MWSCoexistence/Source/subdir.mk
-include framework/Lists/subdir.mk
-include framework/LED/Source/subdir.mk
-include framework/Keyboard/Source/subdir.mk
-include framework/GPIO/subdir.mk
-include framework/FunctionLib/subdir.mk
-include framework/Flash/Internal/subdir.mk
-include framework/DCDC/Source/subdir.mk
-include drivers/subdir.mk
-include board/subdir.mk
-include CMSIS/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := mwa_coordinator_freertos
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: mwa_coordinator_freertos.axf

# Tool invocations
mwa_coordinator_freertos.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -L"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\libs" -Xlinker -Map="mwa_coordinator_freertos.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mcpu=cortex-m0plus -mthumb -T MKW41Z512xxx4_connectivity.ld -L ../source -o "mwa_coordinator_freertos.axf" $(OBJS) $(USER_OBJS) $(LIBS) -lm -lc -lgcc -lnosys -l_crypto_m0 -l802.15.4_mac_06_cm0
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) mwa_coordinator_freertos.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "mwa_coordinator_freertos.axf"; # arm-none-eabi-objcopy -v -O binary "mwa_coordinator_freertos.axf" "mwa_coordinator_freertos.bin" ; # checksum -p MKW41Z512xxx4 -d "mwa_coordinator_freertos.bin";
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
