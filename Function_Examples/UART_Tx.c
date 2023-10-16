#include "stm32f401_uart.h"
#include <stdint.h>

/*
 * This example displays how to configure and initialize the UART2 peripheral in transfer mode, and transfer data
 * using the polling technique. This is a very simple, yet inefficient way to utilize the UART peripheral.
 *
 * For the STM32F401re nucloe-board, UART2 is connected to the ST-Link and therefore does not need any additional
 * hardware to communicate. This is not true, however, for the other UART peripherals and this may also not be true
 * for various other versions of the stm32f4xx nucleo-boards.
 *
 * To run and test this code, some form of serial capture software must be used, such as realterm or teraterm.
 */


//This is the message to be printed.
char message[50] = "Hello World from UART!\n\r";

int main()
{
	UART_Config_t UART2;

	//Configuring UART2 to be in transfer mode and have a badrate of 115200.
	UART_Config(&UART2, USART2, UART_MODE_TX, 115200);

	UART_Init(&UART2);

	while(1)
	{
		//Printing data via polling
		PrintData(&UART2, message);
	}
}


