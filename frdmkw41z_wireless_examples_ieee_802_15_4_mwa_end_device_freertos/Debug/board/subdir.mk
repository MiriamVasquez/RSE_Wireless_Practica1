################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../board/board.c \
../board/clock_config.c \
../board/gpio_pins.c \
../board/pin_mux.c 

C_DEPS += \
./board/board.d \
./board/clock_config.d \
./board/gpio_pins.d \
./board/pin_mux.d 

OBJS += \
./board/board.o \
./board/clock_config.o \
./board/gpio_pins.o \
./board/pin_mux.o 


# Each subdirectory must supply rules for building sources it contributes
board/%.o: ../board/%.c board/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MKW41Z512VHT4_cm0plus -DCPU_MKW41Z512VHT4 -DFSL_RTOS_FREE_RTOS -DFRDM_KW41Z -DFREEDOM -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\source" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\CMSIS" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\OSAbstraction\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\freertos" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\ieee_802.15.4\mac\source\App" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\ieee_802.15.4\mac\interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\ieee_802.15.4\phy\interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\GPIO" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\Keyboard\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\LED\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\SerialManager\Source\SPI_Adapter" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\SerialManager\Source\UART_Adapter" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\SerialManager\Source\I2C_Adapter" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\Flash\Internal" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\common" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\MemManager\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\Messaging\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\Panic\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\RNG\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\SerialManager\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\TimersManager\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\TimersManager\Source" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\ModuleInfo" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\FunctionLib" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\Lists" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\SecLib" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\NVM\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\NVM\Source" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\LowPower\Interface\MKW41Z" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\LowPower\Source\MKW41Z" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\MWSCoexistence\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\drivers" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\DCDC\Interface" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\framework\XCVR\MKW41Z4" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\ieee_802.15.4\phy\source\MKW41Z" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\utilities" -I"C:\Users\Miriam\Documents\MCUXpressoIDE_11.10.0_3148\REDES\frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos\board" -O0 -fno-common -g -gdwarf-4 -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -imacros "C:/Users/Miriam/Documents/MCUXpressoIDE_11.10.0_3148/REDES/frdmkw41z_wireless_examples_ieee_802_15_4_mwa_end_device_freertos/source/app_preinclude.h" -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-board

clean-board:
	-$(RM) ./board/board.d ./board/board.o ./board/clock_config.d ./board/clock_config.o ./board/gpio_pins.d ./board/gpio_pins.o ./board/pin_mux.d ./board/pin_mux.o

.PHONY: clean-board

