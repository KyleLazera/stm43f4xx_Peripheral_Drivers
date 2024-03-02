################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/spi_adxl345_blocking.c \
../Src/spi_adxl345_interrupt.c \
../Src/spi_adxl345_interrupt_multislave.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/main.o \
./Src/spi_adxl345_blocking.o \
./Src/spi_adxl345_interrupt.o \
./Src/spi_adxl345_interrupt_multislave.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/main.d \
./Src/spi_adxl345_blocking.d \
./Src/spi_adxl345_interrupt.d \
./Src/spi_adxl345_interrupt_multislave.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401RETx -DSTM32F4 -DSTM32F401xE -c -I../Inc -I"C:/Users/klaze/Documents/Chip Headers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/klaze/Documents/Chip Headers/CMSIS/Include" -I"C:/Users/klaze/Documents/Chip Headers/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries/STM32F4xx_StdPeriph_Driver/inc" -I"C:/Users/klaze/Documents/Chip Headers/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries/STM32F4xx_StdPeriph_Driver/src" -I"C:/Users/klaze/STM32CubeIDE/SPI_Driver/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/spi_adxl345_blocking.cyclo ./Src/spi_adxl345_blocking.d ./Src/spi_adxl345_blocking.o ./Src/spi_adxl345_blocking.su ./Src/spi_adxl345_interrupt.cyclo ./Src/spi_adxl345_interrupt.d ./Src/spi_adxl345_interrupt.o ./Src/spi_adxl345_interrupt.su ./Src/spi_adxl345_interrupt_multislave.cyclo ./Src/spi_adxl345_interrupt_multislave.d ./Src/spi_adxl345_interrupt_multislave.o ./Src/spi_adxl345_interrupt_multislave.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

