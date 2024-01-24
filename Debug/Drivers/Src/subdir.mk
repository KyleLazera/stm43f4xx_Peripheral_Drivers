################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f401_gpio.c \
../Drivers/Src/stm32f401_i2c.c \
../Drivers/Src/stm32f401_rcc.c \
../Drivers/Src/stm32f401_spi.c \
../Drivers/Src/stm32f401_uart.c 

OBJS += \
./Drivers/Src/stm32f401_gpio.o \
./Drivers/Src/stm32f401_i2c.o \
./Drivers/Src/stm32f401_rcc.o \
./Drivers/Src/stm32f401_spi.o \
./Drivers/Src/stm32f401_uart.o 

C_DEPS += \
./Drivers/Src/stm32f401_gpio.d \
./Drivers/Src/stm32f401_i2c.d \
./Drivers/Src/stm32f401_rcc.d \
./Drivers/Src/stm32f401_spi.d \
./Drivers/Src/stm32f401_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401RETx -DSTM32F4 -DSTM32F401xE -c -I../Inc -I"C:/Users/klaze/Documents/Chip Headers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/klaze/Documents/Chip Headers/CMSIS/Include" -I"C:/Users/klaze/Documents/Chip Headers/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries/STM32F4xx_StdPeriph_Driver/inc" -I"C:/Users/klaze/Documents/Chip Headers/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries/STM32F4xx_StdPeriph_Driver/src" -I"C:/Users/klaze/STM32CubeIDE/SPI_Driver/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f401_gpio.cyclo ./Drivers/Src/stm32f401_gpio.d ./Drivers/Src/stm32f401_gpio.o ./Drivers/Src/stm32f401_gpio.su ./Drivers/Src/stm32f401_i2c.cyclo ./Drivers/Src/stm32f401_i2c.d ./Drivers/Src/stm32f401_i2c.o ./Drivers/Src/stm32f401_i2c.su ./Drivers/Src/stm32f401_rcc.cyclo ./Drivers/Src/stm32f401_rcc.d ./Drivers/Src/stm32f401_rcc.o ./Drivers/Src/stm32f401_rcc.su ./Drivers/Src/stm32f401_spi.cyclo ./Drivers/Src/stm32f401_spi.d ./Drivers/Src/stm32f401_spi.o ./Drivers/Src/stm32f401_spi.su ./Drivers/Src/stm32f401_uart.cyclo ./Drivers/Src/stm32f401_uart.d ./Drivers/Src/stm32f401_uart.o ./Drivers/Src/stm32f401_uart.su

.PHONY: clean-Drivers-2f-Src

