################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Example_Functions/GPIO_EXTI.c \
../Example_Functions/GPIO_Pin_Config.c \
../Example_Functions/GPIO_Port_Init.c 

OBJS += \
./Example_Functions/GPIO_EXTI.o \
./Example_Functions/GPIO_Pin_Config.o \
./Example_Functions/GPIO_Port_Init.o 

C_DEPS += \
./Example_Functions/GPIO_EXTI.d \
./Example_Functions/GPIO_Pin_Config.d \
./Example_Functions/GPIO_Port_Init.d 


# Each subdirectory must supply rules for building sources it contributes
Example_Functions/%.o Example_Functions/%.su Example_Functions/%.cyclo: ../Example_Functions/%.c Example_Functions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401RETx -DSTM32F4 -DSTM32F401xE -c -I../Inc -I"C:/Users/klaze/Documents/Chip Headers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/klaze/Documents/Chip Headers/CMSIS/Include" -I"C:/Users/klaze/Documents/Chip Headers/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries/STM32F4xx_StdPeriph_Driver/inc" -I"C:/Users/klaze/Documents/Chip Headers/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries/STM32F4xx_StdPeriph_Driver/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Example_Functions

clean-Example_Functions:
	-$(RM) ./Example_Functions/GPIO_EXTI.cyclo ./Example_Functions/GPIO_EXTI.d ./Example_Functions/GPIO_EXTI.o ./Example_Functions/GPIO_EXTI.su ./Example_Functions/GPIO_Pin_Config.cyclo ./Example_Functions/GPIO_Pin_Config.d ./Example_Functions/GPIO_Pin_Config.o ./Example_Functions/GPIO_Pin_Config.su ./Example_Functions/GPIO_Port_Init.cyclo ./Example_Functions/GPIO_Port_Init.d ./Example_Functions/GPIO_Port_Init.o ./Example_Functions/GPIO_Port_Init.su

.PHONY: clean-Example_Functions

