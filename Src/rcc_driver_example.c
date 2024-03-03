#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f401_rcc.h"

/*
 * This example will demonstrate how to use multiple functions from the stm32f401_rcc driver
 * to set the system clock, and adjust the frequencies of the AHB, APB1 and APB2.
 * The main goal of this program is to:
 * 		1)Set the HSE as the system clock.
 * 		2)Set the output frequency of the AHB to 84MHz.
 * 		3)Set the output frequency of the APB1 to 21MHz.
 * 		4)Set the output frequency of the APB2 to 42MHz.
 * 		5)Ensure all values are correct by calculating the frequencies using register values.
 *
 * To efficiently use this example program and see each variables value, debug the program and step
 * through each function. This will allow the user to monitor changes in registers as well as changes
 * to variables.
 */

uint32_t pll_set;
uint32_t sysclck, hclck, pclck1, pclck2;

int main()
{
	/*Create an instance of the ClockSource data structure*/
	RCC_ClockFrequency_t ClockSource_PLL_50MHz;

	/*
	 * Set the HSE as the system clock:
	 * Depending on which version of the stm32f401re nucleo-board is being used, or
	 * if an external oscillator has been attached to the OSC_IN pins, the clock source may differ.
	 *
	 * For the version used when originally designing this example, the HSE automatically uses
	 * the ST-LINK MCO as the clock input, which has a frequency of 8MHz. If this frequency differs
	 * it can be changed in the stm32f401_rcc.h file under the 'HSE_Value' definition.
	 */
	RCC_HSEConfig(HSE_Enable);

	/*
	 * To set the system clock to 50MHz the PLL must be used. To additionally adjust the AHB, APB1 and APB2
	 * frequencies, pre-scaler values must be used. These values must be set prior to setting the PLL.
	 *
	 * Calculations for the desired frequency:
	 * Clck_src = 8MHz
	 * pllm = 8
	 * plln = 336
	 * pllp = 4
	 *
	 * Clck_src/pllm = 1MHz (This fits the 1MHz to 2 MHz required range - see reference manual/function description)
	 * (Clck_src/pllm) * plln = 200MHz (Fits the 192MHz to 432MHz range)
	 * ((Clck_src/pllm) * plln)/pllp = 50MHz (which is below 84MHz)
	 *
	 * AHB required pre-scaler = 1
	 * APB1 required pre-scaler = 4
	 * APB2 required pre-scaler = 2
	 */

	RCC_HCLCKConfig(RCC_AHB1Prescaler_1);
	RCC_PCLCK1Config(RCC_APB1Prescaler_4);
	RCC_PCLCK2Config(RCC_APB2Prescaler_2);

	pll_set = RCC_PLLConfig(PLL_HSE, 8, 336, 4);

	/*To check the values are correct calculate the frequencies of the clock based on register values
	 * and assign them to variables which can be checked while debugging.
	 */

	RCC_GetClockFreq(&ClockSource_PLL_50MHz);

	sysclck = ClockSource_PLL_50MHz.SYSCLCK;
	hclck = ClockSource_PLL_50MHz.HCLCK;
	pclck1 = ClockSource_PLL_50MHz.PCLCK1;
	pclck2 = ClockSource_PLL_50MHz.PCLCK2;

}


