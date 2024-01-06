#include "stm32f401_uart.h"
#include "stm32f4xx.h"

/*
 * This is a simple UART peripheral driver with the intention of being used for debugging purposes. The driver is written
 * utilizing the stm32f401_rcc driver and the stm32f401_gpio driver and is written in C. This driver is meant to allow the user
 * access to the UART peripheral by making it easier to initialize the UART, send data, read data and utilize interrupts.
 * This driver was written specifically with the stm32f401re MCU and was tested using the stm32f401re nucleo-board, however,
 * it is compatible with other stm32f4xx MCU's.
 *
 * To test and run the UART driver it is recommended to use some form of serial capture software. While writing the driver I preferred to
 * use Realterm or Teraterm.
 *
 * The Functions within this driver give the user the ability to:
 * 		1) Configure and Initialize a specific UART peripheral.
 * 		2) Transmit multiple bytes of data from the MCU to the laptop.
 * 		3) Read data from the laptop.
 * 		4) Initialize and use interrupts for UART communication.
 *
 * Designed by: Kyle Lazera
 */


/*
 * @brief	This function allows the user to select which UART they would like to use, the mode to use it in,
 * 			and the baud-rate.
 *
 * @note	This function automatically pre-sets the world-length to 8 bits, disables parity and sets over-sampling
 * 			to 16. To change these settings, the user can call the Set functions which are defined further down, or use
 * 			the dot operator.
 *
 * @param	UART_Config: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 *
 * @param	UARTx: Points to the UART_TypeDef data structure and allows the user to select which UART peripheral
 * 			they would like to use.
 * 			The inputs for this function include:
 * 			USART1, USART2 and USART6 (for the stm32f401)
 *
 * @param	mode: Allows the user to select the mode of the UART (receive, transmit or both).
 * 			The inputs for this function include:
 * 			UART_MODE_TX			Transmit only
 * 			UART_MODE_RX			Receive only
 * 			UART_MODE_TXRX			Transmit and Receive
 *
 * @param	baudrate: Allows the user to input their desired baud-rate for the UART peripheral.
 *
 */
void UART_Config(UART_Config_t *UART_Config, USART_TypeDef *UARTx, uint16_t mode, uint32_t baudrate)
{
	UART_Config->UARTx = UARTx;
	UART_Config->Mode = mode;
	UART_Config->BaudRate = baudrate;
	UART_Config->World_Length = UART_WORDLEN_8BITS;
	UART_Config->Parity = UART_PARITY_DISABLED;
	UART_Config->oversampling = UART_OVERSAMPLING_16;
	UART_Config->Stop_Bits = UART_STOPBITS_1;
}

/*
 * @brief	This function activates the UART peripheral specified by the user.
 *
 * @note	This function is not to be used in the main function, but rather assists other functions
 * 			with readability and efficiency.
 *
 * @note	Depending on the peripheral the user wants to use, this function will set the respective GPIO
 * 			port and pins to alternate function, thereby allowing the UART to be accessed.
 *
 * @param	GPIOx: GPIOx: Defines the specific GPIO port, where x can range from A - E, or H.
 *
 * @param	pin: This defines the specific pins associated with the UART peripherals
 *
 * @param	alt_function: Defines the alternate function value to input into the Alternate function register.
 *
 */
void Select_UART_Periph(GPIO_TypeDef *GPIOx, uint8_t pin, AFR_Config_t alt_function)
{
	GPIO_Config_t GPIOx_UARTx_Tx, GPIOx_UARTx_Rx;

	GPIO_Config(&GPIOx_UARTx_Tx, GPIOx, pin, GPIO_AF, GPIO_PushPull, GPIO_LowSpeed, GPIO_PUPD_None);
	GPIO_Init(&GPIOx_UARTx_Tx, alt_function);

	GPIO_Config(&GPIOx_UARTx_Rx, GPIOx, (pin + 1), GPIO_AF, GPIO_PushPull, GPIO_LowSpeed, GPIO_PUPD_None);
	GPIO_Init(&GPIOx_UARTx_Rx, alt_function);
}

/*
 * @brief	This function utilizes the Select_UART_Periph() function to enable the GPIO ports and pins
 * 			based on the peripheral selected by the user.
 *
 * @param	UARTx: Defines which UART peripheral the user chooses, this can be: USART1, USART2 or USART6.
 */
void Enable_UART_Periph(USART_TypeDef *UARTx)
{
	if(UARTx == USART2)
	{

		Select_UART_Periph(GPIOA, Pin2, AF7);
		Select_UART_Periph(GPIOD, Pin5, AF7);

		RCC_APB1Cmd(USART2_Enable, ENABLE);
	}

	else if(UARTx == USART1)
	{
		Select_UART_Periph(GPIOA, Pin9, AF7);
		Select_UART_Periph(GPIOB, Pin6, AF7);

		RCC_APB2Cmd(USART1_Enable, ENABLE);
	}

	else
	{
		Select_UART_Periph(GPIOA, Pin11, AF8);
		Select_UART_Periph(GPIOC, Pin6, AF7);

		RCC_APB2Cmd(USART6_Enable, ENABLE);

	}
}

/*
 * @brief 	This function will calculate the USARTDIV and program it into the
 * 			baud rate register (BRR) based on the desired baud rate the user inputs.
 *
 * 	@note	The Baud Rate register (BRR) is a 32 bit register with the first 15 bits available to
 * 			be written to and the last 15 bits reserved. Within the register, bits 0 - 3 are used
 * 			for the fraction value derived from the equation, and bits 4 - 15 are used for the mantissa
 * 			value.
 *
 * 	@note	Baud Rate Equation:
 * 			Baud = periph_clck / (8 * (2 - OVER8) * USARTDIV)
 * 				**OVER8 is over-sampling, where OVER8 = 0 is over-sampling by 16, and OVER8 = 1 is
 * 		  		  over-sampling by 8.
 *
 * 	@note	Mathematical steps to calculate mantissa and fraction values:
 *			1) Calculate the usartdiv value using equation:
 * 			   usartdiv = periph_clk/(8 * (2 - over8) * baudrate)
 *
 * 			2) This will yield a decimal answer, therefore multiply the peripch_clk
 * 		  	   by 100 to make the decimal portion present in the 32 bit integer.
 *
 * 			3) To calculate the mantissa, divide the usartdiv by 100.
 *
 * 			4) To calculate the fraction, first multiply the mantissa by 100 and subtract this value
 * 		       from usartdiv to isolate the "decimal" portion of usartdiv.
 *
 * 			5) Multiply the new value by the over-sampling (either 16 or 8) and add 50 to round the number
 * 		       up if needed. Lastly divide this value by 100.
 *
 * 		  Example: over8 = 0 (over-sampling = 16), desired baudrate = 115200, periph_clk = 16MHz
 *
 * 		 	1)usartdiv = (16000000 * 100)/(8 * 2 * baudrate)
 * 		   	  usartdiv = 868
 *
 * 		 	2)mantissa = usartdiv/100
 * 		   	  mantissa = 8 or 0x8
 *
 * 		 	3)mantissa << 4 = 0x0008000
 * 		      temporary variable = 0x0 | 0x0008000
 *
 * 		 	4)fraction = (((868 - (8 * 100)) * 16) + 50)/100
 * 		      fraction = 11 or 0xB
 *
 * 		 	5)temporary variable |= 0xB
 * 		     temporary variable = 0x8B
 *
 *@param	USART_Config: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 *
 *@param	ClockSource: Defines the specific clock source for the MCU which includes SYSCLK,
 *			HCLK, PCLK1, PCLK2.
 *
 */
void Set_BaudRate(RCC_ClockFrequency_t *ClockSource, UART_Config_t *UART_Config)
{
	RCC_GetClockFreq(ClockSource);

	uint32_t periph_clk, over8, usartdiv, baudrate;
	uint32_t mantissa_value, fraction_value, tempvariable = 0;

	over8 = (UART_Config->UARTx->CR1) & (UART_CR1_OVER8_Mask);
	baudrate = UART_Config->BaudRate;

	if(UART_Config->UARTx == USART2)
	{
		periph_clk = ClockSource->PCLCK1;
	}

	else
	{
		periph_clk = ClockSource->PCLCK2;
	}

	/*Over-sampling by 16*/
	if(over8 == 0)
	{
		usartdiv = (periph_clk * 100)/(16 * baudrate);
		mantissa_value = usartdiv/100;
		tempvariable |= mantissa_value << 4;

		fraction_value = (((usartdiv - (mantissa_value * 100)) * 16) + 50)/100;
		tempvariable |= fraction_value;
	}

	/*Over-sampling by 8*/
	else
	{
		usartdiv = ((periph_clk * 100)/(8 * baudrate));
		mantissa_value = usartdiv/100;
		tempvariable |= (mantissa_value << 4);
		fraction_value = (((usartdiv - (mantissa_value * 100)) * 8) + 50)/100;
		tempvariable |= fraction_value;
	}

	UART_Config->UARTx->BRR |= tempvariable;
}

/*
 * @brief	Used to adjust the default word length for the UART peripheral.
 *
 * @note	If this value is not changed it will remain at 8 words.
 *
 * @note	This value can also be adjusted without this function and with the use of
 * 			of the dot operator.
 *
 * @param	UART_Config: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 *
 * @param	wordlength: This is the desired word-length the user wishes to set.
 * 			The input options for this argument include:
 * 			UART_WORDLEN_8BITS
 * 			UART_WORDLEN_9BITS
 */
void Set_WordLength(UART_Config_t *UART_Config, uint32_t wordlength)
{
	UART_Config->World_Length = wordlength;
}

/*
 * @brief	Used to set the parity to odd or even for the UART peripheral.
 *
 * @note	By default parity is disabled, using this function however will activate it
 * 			in the UART_Init() function.
 *
 * @note	This value can also be set using the dot operator.
 *
 * @param	UART_Config: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 *
 * @param	parity: This is the desired parity the user wishes to implement.
 * 			The input options for this include:
 * 			UART_PARITY_ENABLED_EVEN
 * 			UART_PARITY_ENABLED_ODD
 */
void Set_Parity(UART_Config_t *UART_Config, uint32_t parity)
{
	UART_Config->Parity = parity;
}

/*
 * @brief	Used to set the over-sampling for the UART peripheral.
 *
 * @note	By default, over-sampling is set to 16.
 *
 * @note	This value can also be set using the dot operator.
 *
 * @param	UART_Config: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 *
 * @paran	oversampling: The desired over-sampling value.
 * 			The input values include:
 * 			UART_OVERSAMPLING_8
 * 			UART_OVERSAMPLING_16
 */
void Set_OverSampling(UART_Config_t *UART_Config, uint32_t oversampling)
{
	UART_Config->oversampling = oversampling;
}

/*
 * @brief	Used to adjust or set the number of stop bits for the UART peripheral.
 *
 * @note	By default this value is set to 1 stop bit;
 *
 * @note	This value can be adjusted using the dot operator.
 *
 * @param	UART_Config: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 *
 * @param	stopbits: The desired number of stop bits.
 * 			The inputs for this argument include:
 * 			UART_STOPBITS_1				1 Stop bit
 * 			UART_STOPBITS_0_5			0.5 Stop bits
 * 			UART_STOPBITS_2				2 Stop bits
 * 			UART_STOPBITS_1_5			1.5 Stop bits
 */
void Set_StopBits(UART_Config_t *UART_Config, uint32_t stopbits)
{
	UART_Config->Stop_Bits = stopbits;
}

/*
 * @brief	Initializes the UART peripheral based on user specified configurations.
 *
 * @note	This function has multiple other functions embedded into it to allow for complete initialization.
 *
 * @param	UART_Config_t: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 */
void UART_Init(UART_Config_t *UART_Config)
{
	RCC_ClockFrequency_t Clock_Source;

	Enable_UART_Periph(UART_Config->UARTx);

	/**Configuring UART**/

	/*Select the Word-Length (8 bits/9 bits)*/
	UART_Config->UARTx->CR1 &= ~UART_WORDLEN_9BITS;
	UART_Config->UARTx->CR1 |= UART_Config->World_Length;

	/*Select over-sampling (8 or 16)*/
	UART_Config->UARTx->CR1 &= ~UART_OVERSAMPLING_8;
	UART_Config->UARTx->CR1 |= UART_Config->oversampling;

	/*Select transmission direction (Tx, Rx or Tx & Rx)*/
	UART_Config->UARTx->CR1 &= ~UART_MODE_TXRX;
	UART_Config->UARTx->CR1 |= UART_Config->Mode;

	/*Select parity (odd or even)*/
	UART_Config->UARTx->CR1 &= ~UART_PARITY_ODD;
	UART_Config->UARTx->CR1 |= UART_Config->Parity;

	/*Select number of stop bits (0.5, 1, 1.5, 2)*/
	UART_Config->UARTx->CR2 &= ~UART_STOPBITS_1_5;
	UART_Config->UARTx->CR2 |= UART_Config->Stop_Bits;

	/*Calculate and set baud-rate*/
	Set_BaudRate(&Clock_Source, UART_Config);

	/*Enable UART*/
	UART_Config->UARTx->CR1 |= UART_CR1_Enable;

}

/*
 * @brief	Transmits a single byte of data through the UART peripheral
 *
 * @note	The function does monitor the word length and whether or not parity is enabled/disabled
 * 			to ensure the word length is correct.
 *
 * @note	This byte can only transmit one byte at a time, however, multiple of these functions can be put together
 * 			to create more complex words.
 *
 * @param	UART_Config: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 *
 * @param	data: This specifies the data the user wishes to transmit.
 */
void WriteByte(UART_Config_t *UART_Config, uint16_t data)
{
	while(!(UART_Config->UARTx->SR) && (UART_SR_TXE_Mask)){}

	//Word length is 9 bits
	if(UART_Config->UARTx->CR1 & UART_WORDLEN_9BITS)
	{
		//If parity is enabled 8 bits of data are sent with the 9th bit being the parity bit
		if((UART_Config->UARTx->CR1 & UART_PARITY_EVEN) || (UART_Config->UARTx->CR1 & UART_PARITY_ODD))
		{
			UART_Config->UARTx->DR = ((uint8_t)data);
		}

		//If parity is disabled then all 9 bits of data will be read as information
		else
		{
			UART_Config->UARTx->DR = (data & 0x1FF);
		}
	}

	//Word length is 8 bits
	else
	{
		//If parity is enabled 7 bits of data will be sent
		if((UART_Config->UARTx->CR1 & UART_PARITY_EVEN) || (UART_Config->UARTx->CR1 & UART_PARITY_ODD))
		{
			UART_Config->UARTx->DR = (data & 0x7F);
		}

		//If parity is disabled all 8 bits will be sent
		else
		{
			UART_Config->UARTx->DR = ((uint8_t)data);
		}
	}
}

/*
 * @brief	An extension of the WriteByte function that allows users to write messages or transfer more complex data.
 *
 * @note	This function relies on the use of the WriteByte function.
 *
 * @param	UART_Config: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 *
 * @param	ptr: This points to a character array and allows the user to pass messages to the UART peripheral.
 */
int PrintData(UART_Config_t *UART_Config, char *ptr)
{
	while(*ptr)
	{
		WriteByte(UART_Config, *ptr++);
	}
	return 1;
}

/*
 * @brief	Reads bytes of data that is transferred into the USART data register.
 *
 * @param	UART_Config_t: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 *
 * @retval	The function returns the value of the data register which is an 8 bit (1 byte) value.
 */
uint8_t ReadByte(UART_Config_t *UART_Config)
{
	while(!((UART_Config->UARTx->SR) & (UART_SR_RXNE_Mask))){}

	return UART_Config->UARTx->DR;
}


/*
 * @brief	Initializes UART peripheral to utilize interrupts during communication.
 *
 * @note	This function allows the user to enable the TXEIE (transmit data register empty interrupt),
 * 			RXNEIE(Read data register not empty interrupt) and the TCIE (Transmission complete interrupt).
 * 			To see which interrupt is better suited for the desired function, check reference manual.
 *
 * @param	UART_Config: This points to the UART_Config_t data structure that holds all of the information for the
 * 			specific UART peripheral.
 *
 * @param	interrupt_line: Allows the user to enter which interrupt they would like to enable.
 * 			The inputs for this argument include:
 * 			 	1)UART_RXNEIE_Enable
 * 				2)UART_TXEIE_Enable
 * 				3)UART_TCIE_Enable
 */
void UART_Interrupt_Init(UART_Config_t *UART_Config, uint8_t interrupt_line)
{
	UART_Config->UARTx->CR1 |= interrupt_line;

	if(UART_Config->UARTx == USART2)
	{
		NVIC_EnableIRQ(USART2_IRQn);
	}

	else if(UART_Config->UARTx == USART1)
	{
		NVIC_EnableIRQ(USART1_IRQn);
	}

	else if(UART_Config->UARTx == USART6)
	{
		NVIC_EnableIRQ(USART6_IRQn);
	}
}


