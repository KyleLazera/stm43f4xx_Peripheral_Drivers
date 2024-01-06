################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/stm32f401_gpio.c \
../Drivers/stm32f401_i2c.c \
../Drivers/stm32f401_rcc.c \
../Drivers/stm32f401_uart.c 

OBJS += \
./Drivers/stm32f401_gpio.o \
./Drivers/stm32f401_i2c.o \
./Drivers/stm32f401_rcc.o \
./Drivers/stm32f401_uart.o 

C_DEPS += \
./Drivers/stm32f401_gpio.d \
./Drivers/stm32f401_i2c.d \
./Drivers/stm32f401_rcc.d \
./Drivers/stm32f401_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/%.o Drivers/%.su Drivers/%.cyclo: ../Drivers/%.c Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401RETx -DSTM32F4 -DSTM32F401xE -c -I../Inc -I"C:/Users/klaze/Documents/Chip Headers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/klaze/Documents/Chip Headers/CMSIS/Include" -I"C:/Users/klaze/Documents/Chip Headers/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries/STM32F4xx_StdPeriph_Driver/inc" -I"C:/Users/klaze/Documents/Chip Headers/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries/STM32F4xx_StdPeriph_Driver/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers

clean-Drivers:
	-$(RM) ./Drivers/stm32f401_gpio.cyclo ./Drivers/stm32f401_gpio.d ./Drivers/stm32f401_gpio.o ./Drivers/stm32f401_gpio.su ./Drivers/stm32f401_i2c.cyclo ./Drivers/stm32f401_i2c.d ./Drivers/stm32f401_i2c.o ./Drivers/stm32f401_i2c.su ./Drivers/stm32f401_rcc.cyclo ./Drivers/stm32f401_rcc.d ./Drivers/stm32f401_rcc.o ./Drivers/stm32f401_rcc.su ./Drivers/stm32f401_uart.cyclo ./Drivers/stm32f401_uart.d ./Drivers/stm32f401_uart.o ./Drivers/stm32f401_uart.su

.PHONY: clean-Drivers

