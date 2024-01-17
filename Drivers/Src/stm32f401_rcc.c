#include <stdint.h>
#include "stm32f401_rcc.h"
#include "stm32f4xx.h"

/*
 * This driver was designed with the purpose of allowing the user to easily access the source
 * clocks as well as change the respective frequencies, therefore, this driver can be used as a base
 * for other peripheral drivers such as a UART, GPIO or I2C driver.
 *
 * This is an RCC driver written specifically for the STM32F401RE MCU nucleo-board and can be used
 * with the stand-alone MCU. This driver gives the user the ability to:
 * 		1)Enable or disable clock access to peripherals.
 * 		2)Calculate the clock frequencies based on register values.
 * 		3)Set or adjust pre-scaler values for the AHB, APB1 and APB2.
 * 		4)Set the system clock as either HSE or HSI.
 * 		5)Set a specified clock frequency using the PLL (Such as the max HCLCK frequency).
 *
 * Designed by: Kyle Lazera
 */

/*
 * @brief	Enables or disables clock access to specified peripheral via AHB1
 *
 * @param	AHB1_Periph: Specific peripheral selected by the user (GPIOx, SPIx, I2Cx etc)
 *
 * @param	State: Specifies whether to enable or disable the peripheral.
 */
void RCC_AHB1Cmd(uint32_t AHB1_Periph, FunctionalState State)
{
	if(State == ENABLE)
	{
		RCC->AHB1ENR |= AHB1_Periph;
	}

	else
	{
		RCC->AHB1ENR &= ~AHB1_Periph;
	}
}


/*
 * @brief	Enables or disables clock access to specified peripheral via APB1
 *
 * @param	APB1_Periph: Specific peripheral selected by the user (GPIOx, SPIx, I2Cx etc)
 *
 * @param	State: Specifies whether to enable or disable the peripheral.
 */
void RCC_APB1Cmd(uint32_t APB1_Periph, FunctionalState State)
{
	if(State == ENABLE)
	{
		RCC->APB1ENR |= APB1_Periph;
	}

	else
	{
		RCC->APB1ENR &= ~APB1_Periph;
	}
}


/*
 * @brief	Enables or disables clock access to specified peripheral via APB2
 *
 * @param	APB2_Periph: Specific peripheral selected by the user (GPIOx, SPIx, I2Cx etc)
 *
 * @param	State: Specifies whether to enable or disable the peripheral.
 */
void RCC_APB2Cmd(uint32_t APB2_Periph, FunctionalState State)
{
	if(State == ENABLE)
	{
		RCC->APB2ENR |= APB2_Periph;
	}

	else
	{
		RCC->APB2ENR &= ~APB2_Periph;
	}
}

/*
 * @brief	Determines clock source that drives SYSLCK, and calculates frequencies of
 * 			SYSCLCK, HCLCK, PCLCK1, PCLKC2.
 *
 * @note	HSI (High Speed Internal) value is default 16MHz
 * 			HSE (High Speed External) value is default 25MHz
 *
 * @note	Frequencies are calculated using pre-scaler values that are loaded into the
 * 			configuration registers of the RCC.
 *
 * @note	The equation to calculate the PLL Clock Frequency is:
 * 			PLL_Freq = Fvco / PLLP
 * 			Fvco = Fpll_input * PLLN/PLLM
 * 			Fpll_input = HSE or HSI input value
 * 			PLLN, PLLM and PLLP are division and multiplication factors present in the PLLCFGR
 *
 * @param	ClockSource: Points to the ClockSource data structure which contains the varibales to hold
 * 			the frequencies of the SYSCLCK, HCLCK, PCLCK1, PCLCK2.
 */
void RCC_GetClockFreq(RCC_ClockFrequency_t *ClockSource)
{
	uint32_t pll_Value = 0, plln = 0, pllm = 2, pllp = 2, pll_src = 0, pll_fvco = 0;

	uint32_t temp_variable, prescaler;

	/*Determine the clock source for the SYSCLCK*/
	if(((RCC->CFGR & RCC_CFGR_SWS_Mask) >> 2) == 0x0)
	{
		ClockSource->SYSCLCK = HSI_Value;
	}

	else if(((RCC->CFGR & RCC_CFGR_SWS_Mask) >> 2) == 0x1)
	{
		ClockSource->SYSCLCK = HSE_Value;
	}

	else if(((RCC->CFGR & RCC_CFGR_SWS_Mask) >> 2) == 0x2)
	{
		/*Determine the input source of the PLL*/
		if(RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_Mask)
		{
			pll_src = HSE_Value;
		}

		else
		{
			pll_src = HSI_Value;
		}

		plln = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Mask) >> 6);
		pllm = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Mask);
		pll_fvco = (pll_src/pllm) * plln;
		pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP_Mask) >> 16) * 2) + 2);
		pll_Value = pll_fvco/pllp;

		ClockSource->SYSCLCK = pll_Value;
	}

	/*Determine the frequency of the HCLCK*/
	temp_variable = ((RCC->CFGR & RCC_CFGR_HPRE_Mask) >> 4);
	prescaler = prescalerTable[temp_variable];
	ClockSource->HCLCK = (ClockSource->SYSCLCK) >> prescaler;

	/*Determine the frequency of the PCLCK1*/
	temp_variable = ((RCC->CFGR & RCC_CFGR_PPRE1_Mask) >> 10);
	prescaler = prescalerTable[temp_variable];
	ClockSource->PCLCK1 = (ClockSource->HCLCK) >> prescaler;

	/*Determine frequency of PCLCK2*/
	temp_variable = ((RCC->CFGR & RCC_CFGR_PPRE2_Mask) >> 13);
	prescaler = prescalerTable[temp_variable];
	ClockSource->PCLCK2 = (ClockSource->HCLCK) >> prescaler;

}

/*
 * @brief	Resets the system clock back to the HSI.
 *
 * @note	The HSI for the STM32F401RE nucleo-board has a frequency
 * 			of 16MHz
 */
void RCC_ResetSYSCLCK()
{
	RCC->CR |= HSI_Enable;
	/*Set the system clock back to HSI*/
	RCC->CFGR &= ~SW_Reset;
	/*Clear all other clocks in CR*/
	RCC->CR &= ~HSE_Enable;
	RCC->CR &= ~HSE_ByPass;
	RCC->CR &= ~PLL_Enable;

}

/*
 * @brief	Sets the frequency of the HCLCK by allowing the user to input
 * 			the desired pre-scaler value.
 *
 * @param	AHB_Prescaler: The pre-scaler value that will be set by the user
 * 			The user can select from:
 * 				RCC_AHB1Prescaler_2			/2
 * 				RCC_AHB1Prescaler_4			/4
 * 				RCC_AHB1Prescaler_8			/8
 * 				RCC_AHB1Prescaler_16		/16
 * 				RCC_AHB1Prescaler_64		/64
 * 				RCC_AHB1Prescaler_128		/128
 * 				RCC_AHB1Prescaler_256		/256
 * 				RCC_AHB1Prescaler_512		/512
 */
void RCC_HCLCKConfig(uint32_t AHB_Prescaler)
{
	RCC->CFGR &= ~RCC_CFGR_HPRE_Mask;
	if(AHB_Prescaler != RCC_AHB1Prescaler_1)
	{
		RCC->CFGR |= AHB_Prescaler;
	}
}

/*
 * @brief	Sets the frequency of the PCLCK1 by allowing the user to input
 * 			the desired pre-scaler value.
 *
 * @note	This pre-scaler must be set so that the PCLCK1 does not exceed 42MHz.
 *
 * @param	APB1_Prescaler: The pre-scaler value that will be set by the user.
 * 			The options for this parameter are:
 * 				RCC_APB1Prescaler_2			/2
 * 				RCC_APB1Prescaler_4			/4
 * 				RCC_APB1Prescaler_8			/8
 * 				RCC_APB1Prescaler_16			/16
 */
void RCC_PCLCK1Config(uint32_t APB1_Prescaler)
{
	RCC->CFGR &= ~RCC_CFGR_PPRE1_Mask;
	RCC->CFGR |= APB1_Prescaler;
}

/*
 * @brief	Sets the frequency of the PCLCK2 by allowing the user to input
 * 			the desired pre-scaler value.
 *
 * @note	This pre-scaler must be set so that the PCLCK2 must not exceed 84MHz.
 *
 * @param	APB2_Prescaler: The pre-scaler value that will be set by the user.
 * 			The options for this parameter are:
 * 				RCC_APB2Prescaler_2			/2
 * 				RCC_APB2Prescaler_4			/4
 * 				RCC_APB2Prescaler_8			/8
 * 				RCC_APB2Prescaler_16		/16
 */
void RCC_PCLCK2Config(uint32_t APB2_Prescaler)
{
	RCC->CFGR &= ~RCC_CFGR_PPRE2_Mask;
	RCC->CFGR |= APB2_Prescaler;
}

/*
 * @brief	Enables the HSE (High speed external) oscillator
 *
 * @note	The function allows the HSE to be selected as HSE bypass,
 * 			which requires an external clock source on the OSC_IN (PH0) I/O pins.
 *
 * @note	The function also allows for the HSE to be selected as an external crystal
 * 			oscillator.
 *
 * @note	The function ensures the HSERDY flag is raised before exiting the function
 *
 * @param	HSE_State: Whether the HSE will be an external clock source or a crystal.
 * 			This parameter can have the inputs:
 * 				HSE_Bypass			External Clock (1MHz to 50MHz)
 * 				HSE_Enable			Crystal oscillator (4MHz to 26MHz)
 */
void RCC_HSEConfig(uint32_t HSE_State)
{

	if(HSE_State == HSE_ByPass)
	{
		RCC->CR |= HSE_State;
		/*Set HSE as system clock source*/
		RCC->CFGR &= ~SW_Reset;
		RCC->CFGR |= HSE_SW_Enable;
		/*Disable HSI clock source*/
		RCC->CR &= ~RCC_CR_HSION_Mask;
	}

	else
	{
		RCC->CR |= HSE_State;
		/*Set HSE as system clock source*/
		RCC->CFGR &= ~SW_Reset;
		RCC->CFGR |= HSE_SW_Enable;
		/*Disable HSI clock source*/
		RCC->CR &= ~RCC_CR_HSION_Mask;
	}

	while(!(RCC->CR & HSE_HSERDY_Flag)){}
}

/*
 * @brief	Enables and allows the user to configure the PLL (Phase locked loop) to achieve
 * 			higher clock frequencies.
 *
 * @note	The PLL clock output can be calculated by (from reference manual):
 * 			Fvco_clock = PLL_ClckSrc * (PLLN/PLLM)
 * 			PLLoutput = Fvco_clock/PLLP
 *
 * @note	To meet the clock frequency limitations listed below, ensure that the correct pre-scaler
 * 			values are set for the AHB, APB1 and APB2. This can be done using the:
 * 			RCC_HCLCKConfig(), RCC_PCLCK1Config(), RCC_PCLCK2Config() functions.
 * 			These values must be set or the function will not operate.
 *
 *
 * @note	The PLL has multiple limitations that must be adhered to when entering respective
 * 			multiplicative and division factors. The limitations listed are specific to the
 * 			STM32F401RE micro-controller:
 * 			Clock frequency Limits:
 * 			1) The AHB cannot exceed 84MHz
 * 			2) The APB1 cannot exceed 42MHz
 * 			3) The APB2 cannot exceed 84MHz
 *
 * 			Further Limitations when inputting values:
 * 			1) (PLL_ClckSrc/PLLM) = vco_in must be within 1MHz - 2MHz
 * 			2) PLLM can range from 2 - 63
 * 			3) Fvco_clock = ((PLL_ClckSrc/PLLM) * PLLN) must be within 192MHz - 432MHz
 * 			4) PLLN can range from 192 to 432
 * 			5) Fvco_clock / PLLP cannot exceed 84MHz
 * 			6) PLLP values can be: 2, 4, 6, 8
 *
 * @note	Prior to implementing this function, ensure the calculations are correct and
 * 			meet all the limitations.
 *
 * @note	If the HSE is selected as the driving clock source, ensure it is configured
 * 			first by using the HSEConfig() function.
 *
 * @note	This function has a return value to assist in debugging. When using the function,
 * 			assign it to a variable and monitor the value of the variable to ensure it returns
 * 			the correct value. See below for value meanings.
 *
 * @param	Clcksrc: This dictates what the driving clock source for the PLL will be
 * 			This parameter can have the values:
 * 			PLL_HSI				HSI is the driving clock source
 * 			PLL_HSE				HSE is the driving clock source
 *
 * @param	pllm: This is one of the division factors that will divide the clock source directly.
 * 			This parameter can have values ranging from 2 to 63.
 *
 * @param	plln: This is a multiplicative factor for the source clock and pllm.
 * 			This parameter can range from 192 to 432.
 *
 * @param	pllp: This is the second division that divides the Fvco_clock value.
 * 			This parameter can have values of: 2, 4, 6, 8
 *
 * @retval	PLL_ErrorHandler_t is an enumeration declared in the stm32f401_rcc.h file.
 * 			The enumeration is used to debug the function, and allows the user to identify is
 * 			any of the limitations are not met.
 * 			The return value will return:
 * 				1)PLL_Set = 0, indicating the PLL is set successfully.
 * 				2)AHB_ExceedFreq = 1, indicating the AHB bus exceeds 84MHz.
 * 				3)APB1_ExceedFreq = 2, indicating the APB1 bus has exceeded 42MHz.
 * 				4)VCO_Input = 3, indicating the vco_input is less than 1MHz or greater then 2MHz.
 * 				5)VCO_Output = 4, indicating the vco_output is not within 192MHz and 432MHz.
 *
 */
PLL_ErrorHandler_t RCC_PLLConfig(uint32_t Clocksrc, uint8_t pllm, uint16_t plln, uint8_t pllp)
{
	uint32_t clcksrc, vco_in, vco_out, pll_out;
	uint32_t pclck1_freq, pclck1_prescaler;
	uint32_t temp_variable, flash_read;

	RCC->CR &= ~PLL_Enable;

	/*Setting PLL clock source*/
	if(Clocksrc == PLL_HSE)
	{
		RCC->PLLCFGR |= PLL_HSE;
		clcksrc = HSE_Value;
	}

	else
	{
		RCC->PLLCFGR |= PLL_HSI;
		clcksrc = HSI_Value;
	}

	/*Setting the pllm division factor, and error checking that vco_input ranges from 1MHz - 2MHz.*/
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Mask;
	RCC->PLLCFGR |= (pllm << RCC_PLLCFGR_PLLM_Pos);
	vco_in = clcksrc / pllm;
	if(vco_in < 1000000 || vco_in > 2000000)
	{
		return VCO_Input;
	}

	/*Setting plln multiplicative factor, and checking vco_out ranges from 194MHz - 432MHz.*/
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Mask;
	RCC->PLLCFGR |= (plln << RCC_PLLCFGR_PLLN_Pos);
	vco_out = vco_in * plln;
	if(vco_out < 192000000 || vco_out > 432000000)
	{
		return VCO_Output;
	}

	/*Setting pllp division factor and checking if pll_out is less than 84MHz*/
	pll_out = vco_out / pllp;
	pllp = (pllp/2) - 1;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_Mask;
	RCC->PLLCFGR |= (pllp << RCC_PLLCFGR_PLLP_Pos);
	if(pll_out > 84000000)
	{
		return AHB_ExceedFreq;
	}

	/*Enable access to the PWR peripheral to ensure the regulatory voltage is scaled*/
	RCC_APB1Cmd(PWR_Enable, ENABLE);
	while(!((RCC->APB1ENR & RCC_APB1ENR_PWREnable) >> 28));
	PWR->CR |= PWR_CR_VOS_10;
	RCC_APB1Cmd(PWR_Enable, DISABLE);

	/*Set 5 wait states to allow CPU to read FLASH memory*/
	FLASH->ACR &= ~(FLASH_ACR_Latency_Mask);
	FLASH->ACR |= FLASH_ACR_Latency_5WS;
	flash_read = FLASH->ACR;


	/*Error checking to ensure the APB1 bus does not exceed 42MHz.*/
	temp_variable = ((RCC->CFGR & RCC_CFGR_HPRE_Mask) >> 4);
	pll_out = pll_out >> (prescalerTable[temp_variable]);
	temp_variable = ((RCC->CFGR & RCC_CFGR_PPRE1_Mask) >> 10);
	pclck1_prescaler = prescalerTable[temp_variable];
	pclck1_freq = pll_out >> pclck1_prescaler;
	if(pclck1_freq > 42000000)
	{
		return APB1_ExceedFreq;
	}


	/*Setting PLL as the main clock*/
	RCC->CR |= PLL_Enable;
	while(!(RCC->CR & PLL_PLLRDY_FLAG)){};

	RCC->CFGR &= ~SW_Reset;
	RCC->CFGR |= PLL_SW_Enable;

	return PLL_Set;

}




