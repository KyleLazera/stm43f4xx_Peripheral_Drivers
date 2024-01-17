

#ifndef STM32F401_GPIO_H_
#define STM32F401_GPIO_H_

#include <stdint.h>
#include "stm32f401_rcc.h"
#include "stm32f4xx.h"

typedef enum
{
	AF0 = 0x0,
	AF1 = 0x1,
	AF2 = 0x2,
	AF3 = 0x3,
	AF4 = 0x4,
	AF5 = 0x5,
	AF6 = 0x6,
	AF7 = 0x7,
	AF8 = 0x8,
	AF9 = 0x9,
	AF10 = 0xA,
	AF11 = 0xB,
	AF12 = 0xC,
	AF13 = 0xD,
	AF14 = 0xE,
	AF15 = 0xF
}AFR_Config_t;

typedef struct
{
	uint8_t GPIO_Pin;
	uint8_t GPIO_MODE;
	uint8_t GPIO_OTYPE;
	uint8_t GPIO_OSPEED;
	uint8_t GPIO_PUPD;
	GPIO_TypeDef *GPIO_Port;
}GPIO_Config_t;

/********** GPIO Configuration Macros **********/
/***** Mode Register *****/
#define GPIO_Mode_Reset					0x3
#define GPIO_Input						0x0
#define GPIO_Output						0x1
#define GPIO_AF							0x2
#define GPIO_Analog						0x3

/***** Output Type Register *****/
#define GPIO_OType_Reset				0x1
#define GPIO_PushPull					0x0
#define GPIO_OpenDrain					0x1

/***** Output Speed Register *****/
#define GPIO_OSpeed_Reset				0x1
#define GPIO_LowSpeed					0x0
#define GPIO_MediumSpeed				0x1
#define GPIO_HighSpeed					0x2
#define GPIO_VeryHighSpeed				0x3

/***** Pullup/Pulldown Register *****/
#define GPIO_PUPD_Reset					0x3
#define GPIO_PUPD_None					0x0
#define GPIO_PullUp						0x1
#define GPIO_PullDown					0x2

/***** Input Data Register Macros *****/
#define GPIO_IDR_Mask					(0x1UL)

/***** Pin Definitions *****/
#define Pin0							0x0
#define Pin1							0x1
#define Pin2							0x2
#define Pin3							0x3
#define Pin4							0x4
#define Pin5							0x5UL
#define Pin6							0x6
#define Pin7							0x7
#define Pin8							0x8
#define Pin9							0x9
#define Pin10							0xA
#define Pin11							0xB
#define Pin12							0xC
#define Pin13							0xD
#define Pin14							0xE
#define Pin15							0xF
#define AllPins							0x10

/********** GPIO Output States **********/
#define GPIO_Write						0x1
#define GPIO_Reset						0x2
#define GPIO_Toggle						0x3

/********** GPIO Lock Register **********/
#define LCKR_1_Pin13_Pin14				0x00016000
#define LCKR_0_Pin13_Pin14				0x00006000

/********** Interrupt Configuration **********/
#define RCC_APB2ENR_SYSCFGEN_Pos		(14U)
#define RCC_APB2ENR_SYSCFGEN			(0x1UL << RCC_APB2ENR_SYSCFGEN_Pos)
#define EXTI_IMR_Set					(0x1UL)
#define EXTI_Trigger_Set				(0x1UL)

/********** IRQ Macros **********/
#define EXTI_PortA							0x0UL
#define EXTI_PortB							0x1UL
#define EXTI_PortC							0x2UL
#define EXTI_PortD							0x3UL
#define EXTI_PortE							0x4UL
#define EXTI_PortH							0x7UL

/********** EXTI_Trigger Macros **********/
#define EXTI_RisingTrigger					0x0
#define EXTI_FallingTrigger					0x1
#define EXTI_Rising_FallingTrigger			0x2

/********** Functions **********/
void GPIO_Config(GPIO_Config_t *GPIO_Config, GPIO_TypeDef *Port, uint8_t Pin, uint8_t Mode, uint8_t OType, uint8_t OSpeed, uint8_t PUPD);
void GPIO_Init(GPIO_Config_t *GPIO_Config, AFR_Config_t alt_function);
void GPIO_AlternateFunctionConfig(GPIO_Config_t *GPIO_Config, AFR_Config_t alt_function);
void GPIO_PeriphClck(GPIO_TypeDef *GPIOx, FunctionalState state);
void GPIO_WritePin(GPIO_Config_t *GPIO_Config, uint8_t State);
void GPIO_WritePort(GPIO_Config_t *GPIO_Config, uint16_t word);
uint8_t GPIO_ReadPin(GPIO_Config_t *GPIO_Config);
uint16_t GPIO_ReadPort(GPIO_Config_t *GPIO_Config);
void GPIO_ConfigLEDPA5(uint8_t State);
uint8_t GPIO_ConfigButtonPC13(void);
void GPIO_EXTIConfig(uint8_t port, uint8_t pin, uint8_t edge_trigger);


#endif /* STM32F401_GPIO_H_ */
