#include "stm32f401_uart.h"
#include <stdint.h>

/*
 * This example displays another form of UART transmission using interrupts. This is a significantly more effective method than
 * polling, and is recommended for more advanced programs. In this program, the interrupt handler is called whenever the TXE flag
 * (Transfer data register empty) is raised.
 *
 * This is a simple example where the output will be the message continously printed to the serial mointor, however, it can be used
 * much more efficiently to debug. For example, using this message to indicate a push-button is pressed.
 */

UART_Config_t UART2;

uint8_t read_data;

char message[50] = "Hello World From UART!\r\n";

int main()
{

	//Configuring UART2 to be in reciever mode and have a badrate of 115200.
	UART_Config(&UART2, USART2, UART_MODE_TX, 115200);

	UART_Init(&UART2);

	//Initiliaing the UART interrupt - This function must be called after UART_Init()
	UART_Interrupt_Init(&UART2, UART_TXEIE_Enable);

	while(1)
	{

	}
}

//Interrupt Handler Function
void USART2_IRQHandler(void)
{
	//Monitoring the TXE flag to ensure it is raised
	if((USART2->SR) & (UART_SR_TXE_Mask))
	{
		PrintData(&UART2, message);
	}
}




