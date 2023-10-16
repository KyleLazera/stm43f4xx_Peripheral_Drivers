#include "stm32f401_uart.h"
#include <stdint.h>

/*
 * This is a basic example of how to use the uart driver to initialize the UART to recieve data and using interrupts.
 * In this program, the UART peripheral will read user input, and depending on what the input is, it will either turn on
 * or off the on-board LED.
 *
 * This example uses functions from the stm32f401_gpio driver, which allows for easy initialization and use of the on-board
 * LED.
 */

UART_Config_t UART2;

uint8_t temporary_variable;

int main()
{

	//Configuring UART2 to be in reciever mode and have a badrate of 115200.
	UART_Config(&UART2, USART2, UART_MODE_RX, 115200);

	UART_Init(&UART2);

	//Initiliaing the UART interrupt - This function must be called after UART_Init()
	UART_Interrupt_Init(&UART2, UART_RXNEIE_Enable);

	while(1)
	{

	}
}

//Interrupt Handler Function
void USART2_IRQHandler(void)
{
	//Monitoring the TXE flag to ensure it is raised
	if((USART2->SR) & (UART_SR_RXNE_Mask))
	{
		//Read the input data from the user
		temporary_variable = ReadByte(&UART2);

		if(temporary_variable == '1')
		{
			//Turn the on-board LED on
			GPIO_ConfigLEDPA5(GPIO_Write);
		}

		else
		{
			//Turn the on-board LED off
			GPIO_ConfigLEDPA5(GPIO_Reset);
		}

	}
}

