#include "stm32f401_uart.h"
#include <stdint.h>

/*
 * This is a basic echo program that utilzes interrupts and has the UART set to both transmit and recieve data.
 * In this program, whatever values are read from the UART are then transmitted back to the screen, which echos
 * the input data from the user.
 */

UART_Config_t UART2;

uint8_t temporary_variable;

int main()
{

	//Configuring UART2 to be in reciever mode and have a badrate of 9600.
	UART_Config(&UART2, USART2, UART_MODE_TXRX, 9600);

	UART_Init(&UART2);

	//Initiliaing the UART interrupt - This function must be called after UART_Init()
	UART_Interrupt_Init(&UART2, UART_RXNEIE_Enable);


	while(1)
	{

	}
}

//Interrupt handler function
void USART2_IRQHandler(void)
{
	if((USART2->SR) && (UART_SR_RXNE_Mask))
	{
		//Read the input data from the user
		temporary_variable = ReadByte(&UART2);

		//Print the inputted data to the screen
		WriteByte(&UART2, temporary_variable);

	}
}


