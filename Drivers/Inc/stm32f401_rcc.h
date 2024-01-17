

#ifndef STM32F401_RCC_H_
#define STM32F401_RCC_H_

#include <stdint.h>
#include <stm32f4xx.h>

static volatile uint16_t prescalerTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

typedef enum
{
	PLL_Set = 0,
	AHB_ExceedFreq = 1,
	APB1_ExceedFreq = 2,
	VCO_Input = 3,
	VCO_Output = 4
}PLL_ErrorHandler_t;

typedef struct
{
	uint32_t SYSCLCK;
	uint32_t HCLCK;
	uint32_t PCLCK1;
	uint32_t PCLCK2;
}RCC_ClockFrequency_t;

/********** RCC Clock Source Default Frequencies **********/
#define HSI_Value					16000000
#define HSE_Value					8000000

/********** AHB1 Peripheral Clock Commands **********/
#define GPIOA_Enable				((uint32_t)0x00000001)
#define GPIOB_Enable				((uint32_t)0x00000002)
#define GPIOC_Enable				((uint32_t)0x00000004)
#define GPIOD_Enable				((uint32_t)0x00000008)
#define GPIOE_Enable				((uint32_t)0x00000010)
#define GPIOH_Enable				((uint32_t)0x00000080)
#define CRC_Enable					((uint32_t)0x00001000)
#define DMA1_Enable					((uint32_t)0x00200000)
#define DMA2_Enable					((uint32_t)0x00400000)

/********** APB1 Peripheral Clock Commands **********/
#define TIM2_Enable					((uint32_t)0x00000001)
#define TIM3_Enable					((uint32_t)0x00000002)
#define TIM4_Enable					((uint32_t)0x00000004)
#define TIM5_Enable					((uint32_t)0x00000008)
#define WWDG_Enable					((uint32_t)0x00000800)
#define SPI2_Enable					((uint32_t)0x00004000)
#define SPI3_Enable					((uint32_t)0x00008000)
#define USART2_Enable				((uint32_t)0x00020000)
#define I2C1_Enable					((uint32_t)0x00200000)
#define I2C2_Enable					((uint32_t)0x00400000)
#define I2C3_Enable					((uint32_t)0x00800000)
#define PWR_Enable					((uint32_t)0x10000000)

/********** APB2 Peripheral Clock Commands **********/
#define TIM1_Enable					((uint32_t)0x00000001)
#define USART1_Enable				((uint32_t)0x00000010)
#define USART6_Enable				((uint32_t)0x00000020)
#define ADC1_Enable					((uint32_t)0x00000100)
#define SDIO_Enable					((uint32_t)0x00000800)
#define SPI1_Enable					((uint32_t)0x00001000)
#define SPI4_Enable					((uint32_t)0x00002000)
#define SYSCFG_Enable				((uint32_t)0x00004000)
#define TIM9_Enable					((uint32_t)0x00010000)
#define TIM10_Enable				((uint32_t)0x00020000)
#define TIM11_Enable				((uint32_t)0x00040000)

/********** RCC CFGR Register Masks **********/
#define RCC_CFGR_SWS_Pos			(2U)
#define RCC_CFGR_SWS_Mask			(uint32_t)(0x3UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_HPRE_Pos			(4U)
#define RCC_CFGR_HPRE_Mask			(0xFUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_PPRE1_Pos			(10U)
#define RCC_CFGR_PPRE1_Mask			(0x7UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE2_Pos			(13U)
#define RCC_CFGR_PPRE2_Mask			(0x7UL << RCC_CFGR_PPRE2_Pos)
#define RCC_PLLCFGR_PLLSRC_Pos		(22U)
#define RCC_PLLCFGR_PLLSRC_Mask		(0x1UL << RCC_PLLCFGR_PLLSRC_Pos)
#define RCC_PLLCFGR_PLLN_Pos		(6U)
#define RCC_PLLCFGR_PLLN_Mask		(0x1FFUL << RCC_PLLCFGR_PLLN_Pos)
#define RCC_PLLCFGR_PLLM_Pos		(0U)
#define RCC_PLLCFGR_PLLM_Mask		(0x3FUL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLP_Pos		(16U)
#define RCC_PLLCFGR_PLLP_Mask		(0x3UL << RCC_PLLCFGR_PLLP_Pos)
#define RCC_CR_HSION_Pos			(0U)
#define RCC_CR_HSION_Mask			(0x1UL << RCC_CR_HSION_Pos)
#define RCC_CR_HSEON_Pos			(16U)
#define RCC_CR_HSEON_Mask			(0x1UL << RCC_CR_HSEON_Pos)


/********** RCC HCLCK Pre-scaler Values **********/
#define RCC_CFGR_HPRE_Pos			(4U)
#define RCC_AHB1Prescaler_1			(0x1UL << RCC_CFGR_HPRE_Pos)
#define RCC_AHB1Prescaler_2			(0x8UL << RCC_CFGR_HPRE_Pos)
#define RCC_AHB1Prescaler_4			(0x9UL << RCC_CFGR_HPRE_Pos)
#define RCC_AHB1Prescaler_8			(0xAUL << RCC_CFGR_HPRE_Pos)
#define RCC_AHB1Prescaler_16		(0xBUL << RCC_CFGR_HPRE_Pos)
#define RCC_AHB1Prescaler_64		(0xCUL << RCC_CFGR_HPRE_Pos)
#define RCC_AHB1Prescaler_128		(0xDUL << RCC_CFGR_HPRE_Pos)
#define RCC_AHB1Prescaler_256		(0xEUL << RCC_CFGR_HPRE_Pos)
#define RCC_AHB1Prescaler_512		(0xFUL << RCC_CFGR_HPRE_Pos)

/********** RCC PCLCK1/PCLCK2 Pre-scaler Values **********/
#define RCC_CFGR_PPRE1_Pos			(10U)
#define RCC_APB1Prescaler_1			(0x1UL << RCC_CFGR_PPRE1_Pos)
#define RCC_APB1Prescaler_2			(0x4UL << RCC_CFGR_PPRE1_Pos)
#define RCC_APB1Prescaler_4			(0x5UL << RCC_CFGR_PPRE1_Pos)
#define RCC_APB1Prescaler_8			(0x6UL << RCC_CFGR_PPRE1_Pos)
#define RCC_APB1Prescaler_16		(0x7UL << RCC_CFGR_PPRE1_Pos)

#define RCC_CFGR_PPRE2_Pos			(13U)
#define RCC_APB2Prescaler_1			(0x1UL << RCC_CFGR_PPRE2_Pos)
#define RCC_APB2Prescaler_2			(0x4UL << RCC_CFGR_PPRE2_Pos)
#define RCC_APB2Prescaler_4			(0x5UL << RCC_CFGR_PPRE2_Pos)
#define RCC_APB2Prescaler_8			(0x6UL << RCC_CFGR_PPRE2_Pos)
#define RCC_APB2Prescaler_16		(0x7UL << RCC_CFGR_PPRE2_Pos)

/********** RCC SYSCLCK Configuration **********/

#define HSI_Pos					(0U)
#define HSI_HSIRDY_Pos			(1U)
#define HSI_Enable				(0x1UL << HSI_Pos)
#define HSI_HSIRDY_Flag			(0x1UL << HSI_HSIRDY_Pos)

#define HSE_EnablePos			(16U)
#define HSE_HSERDY_Pos			(17U)
#define HSE_ByPassPos			(18U)
#define HSE_ByPass				(0x1UL << HSE_ByPassPos)
#define HSE_Enable				(0x1UL << HSE_EnablePos)
#define HSE_HSERDY_Flag			(0x1UL << HSE_HSERDY_Pos)

#define PLL_Enable_Pos			(24U)
#define PLL_PLLRDY_Pos			(25U)
#define PLL_Enable				(0x1UL << PLL_Enable_Pos)
#define PLL_PLLRDY_FLAG			(0x1UL << PLL_PLLRDY_Pos)
#define RCC_PLLCFGR_PLLSRC_Pos	(22U)
#define PLL_HSI					(0x0UL << RCC_PLLCFGR_PLLSRC_Pos)
#define PLL_HSE					(0x1UL << RCC_PLLCFGR_PLLSRC_Pos)

#define SW_Pos					(0U)
#define SW_Reset				(0x3 << SW_Pos)
#define HSI_SW_Enable			(0x0 << SW_Pos)
#define HSE_SW_Enable			(0x1 << SW_Pos)
#define PLL_SW_Enable			(0x2 << SW_Pos)

/********** Power Access **********/

#define RCC_APB1ENR_PWREN_Pos		(28U)
#define RCC_APB1ENR_PWREnable		(0x1UL << RCC_APB1ENR_PWREN_Pos)
#define PWR_CR_VOS_Pos				(14U)
#define PWR_CR_VOS_01				(0x1UL << PWR_CR_VOS_Pos)
#define PWR_CR_VOS_10				(0x2UL << PWR_CR_VOS_Pos)
#define PWR_CSR_VOSRDY_Pos			(14U)
#define PWR_CSR_VOSReady			(0x1UL << PWR_CSR_VOSRDY_Pos)


/********** Wait State/Flash Memory Latency **********/

#define FLASH_ACR_Latency_Pos	(0U)
#define FLASH_ACR_Latency_Mask	(0xFUL << FLASH_ACR_Latency_Pos)
#define FLAH_ACR_Latency_0WS	(0x0UL << FLASH_ACR_Latency_Pos)
#define FLASH_ACR_Latency_1WS	(0x1UL << FLASH_ACR_Latency_Pos)
#define FLASH_ACR_Latency_2WS	(0x2UL << FLASH_ACR_Latency_Pos)
#define FLASH_ACR_Latency_5WS	(0x5UL << FLASH_ACR_Latency_Pos)


/********** Functions **********/

void RCC_AHB1Cmd(uint32_t AHB1_Periph, FunctionalState State);
void RCC_APB1Cmd(uint32_t APB1_Periph, FunctionalState State);
void RCC_APB2Cmd(uint32_t APB2_Periph, FunctionalState State);
void RCC_GetClockFreq(RCC_ClockFrequency_t *ClockSource);
void RCC_HCLCKConfig(uint32_t AHB_Prescaler);
void RCC_PCLCK1Config(uint32_t APB1_Prescaler);
void RCC_PCLCK2Config(uint32_t APB2_Prescaler);
void RCC_HSEConfig(uint32_t HSE_State);
void RCC_ResetSYSCLCK();
PLL_ErrorHandler_t RCC_PLLConfig(uint32_t Clocksrc, uint8_t pllm, uint16_t plln, uint8_t pllp);

#endif /* STM32F401_RCC_H_ */
