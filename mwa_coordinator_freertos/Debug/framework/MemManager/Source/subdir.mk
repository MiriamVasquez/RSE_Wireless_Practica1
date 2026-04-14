################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../framework/MemManager/Source/MemManager.c 

C_DEPS += \
./framework/MemManager/Source/MemManager.d 

OBJS += \
./framework/MemManager/Source/MemManager.o 


# Each subdirectory must supply rules for building sources it contributes
framework/MemManager/Source/%.o: ../framework/MemManager/Source/%.c framework/MemManager/Source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_MKW41Z512VHT4_cm0plus -DCPU_MKW41Z512VHT4 -DFSL_RTOS_FREE_RTOS -DFRDM_KW41Z -DFREEDOM -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\source" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\CMSIS" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\OSAbstraction\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\freertos" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\ieee_802.15.4\mac\source\App" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\ieee_802.15.4\mac\interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\ieee_802.15.4\phy\interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\GPIO" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\Keyboard\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\LED\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\SerialManager\Source\SPI_Adapter" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\SerialManager\Source\UART_Adapter" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\SerialManager\Source\I2C_Adapter" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\Flash\Internal" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\common" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\MemManager\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\Messaging\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\Panic\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\RNG\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\SerialManager\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\TimersManager\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\TimersManager\Source" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\ModuleInfo" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\FunctionLib" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\Lists" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\SecLib" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\MWSCoexistence\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\drivers" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\DCDC\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\framework\XCVR\MKW41Z4" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\ieee_802.15.4\phy\source\MKW41Z" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\utilities" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\mwa_coordinator_freertos\board" -O0 -fno-common -g -gdwarf-4 -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -imacros "C:/Users/Miriam/Documents/MCUXpressoIDE_11.10.0_3148/REDES/mwa_coordinator_freertos/source/app_preinclude.h" -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-framework-2f-MemManager-2f-Source

clean-framework-2f-MemManager-2f-Source:
	-$(RM) ./framework/MemManager/Source/MemManager.d ./framework/MemManager/Source/MemManager.o

.PHONY: clean-framework-2f-MemManager-2f-Source

