################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Function_Examples/UART_Rx.c \
../Function_Examples/UART_Rx_Interrupt.c \
../Function_Examples/UART_Tx.c \
../Function_Examples/UART_TxRx.c \
../Function_Examples/UART_Tx_Interrupt.c 

OBJS += \
./Function_Examples/UART_Rx.o \
./Function_Examples/UART_Rx_Interrupt.o \
./Function_Examples/UART_Tx.o \
./Function_Examples/UART_TxRx.o \
./Function_Examples/UART_Tx_Interrupt.o 

C_DEPS += \
./Function_Examples/UART_Rx.d \
./Function_Examples/UART_Rx_Interrupt.d \
./Function_Examples/UART_Tx.d \
./Function_Examples/UART_TxRx.d \
./Function_Examples/UART_Tx_Interrupt.d 


# Each subdirectory must supply rules for building sources it contributes
Function_Examples/%.o Function_Examples/%.su Function_Examples/%.cyclo: ../Function_Examples/%.c Function_Examples/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401RETx -DSTM32F4 -DSTM32F401xE -c -I../Inc -I"C:/Users/klaze/Documents/Chip Headers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/klaze/Documents/Chip Headers/CMSIS/Include" -I"C:/Users/klaze/Documents/Chip Headers/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries/STM32F4xx_StdPeriph_Driver/inc" -I"C:/Users/klaze/Documents/Chip Headers/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries/STM32F4xx_StdPeriph_Driver/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Function_Examples

clean-Function_Examples:
	-$(RM) ./Function_Examples/UART_Rx.cyclo ./Function_Examples/UART_Rx.d ./Function_Examples/UART_Rx.o ./Function_Examples/UART_Rx.su ./Function_Examples/UART_Rx_Interrupt.cyclo ./Function_Examples/UART_Rx_Interrupt.d ./Function_Examples/UART_Rx_Interrupt.o ./Function_Examples/UART_Rx_Interrupt.su ./Function_Examples/UART_Tx.cyclo ./Function_Examples/UART_Tx.d ./Function_Examples/UART_Tx.o ./Function_Examples/UART_Tx.su ./Function_Examples/UART_TxRx.cyclo ./Function_Examples/UART_TxRx.d ./Function_Examples/UART_TxRx.o ./Function_Examples/UART_TxRx.su ./Function_Examples/UART_Tx_Interrupt.cyclo ./Function_Examples/UART_Tx_Interrupt.d ./Function_Examples/UART_Tx_Interrupt.o ./Function_Examples/UART_Tx_Interrupt.su

.PHONY: clean-Function_Examples

