#include "stm32f401_uart.h"
#include <stdint.h>

/*
 * This example displays how to set and use the UART peripheral in reciever(Rx) mode, using polling. In this mode,
 * the user will input some character or value into the Serialcapture software and the UART will read this data and store
 * it in the data register.
 *
 * Using the functions defined in the uart driver, the ReadByte function reads and returns the value from the register.
 * In this program, the variable 'data' holds the information, and I utilized a simple function from the GPIO driver that automatically
 * configures the on-board periphal. When the 'data' varibale is set to 1, the on-board LED turns on and when the 'data' varibale is changed,
 * the LED turns off. This is one way of simple debugging of the UART driver.
 *
 * Additionally, this code can be debugged, and the 'data' variable can be monitored by the live expressions window which will show the user
 * the values the UART is reading from the data register.
 */

uint8_t data;

int main()
{
	UART_Config_t UART2;

	//Configuring UART2 to be in reciever mode and have a badrate of 115200.
	UART_Config(&UART2, USART2, UART_MODE_RX, 115200);

	UART_Init(&UART2);

	while(1)
	{
		//Read data via polling
		data = ReadByte(&UART2);

		//If input data is set to 1
		if(data == '1')
		{
			//Turn on the on-board LED
			GPIO_ConfigLEDPA5(GPIO_Write);
		}

		//If input data is not set to 1
		else
		{
			//Turn off the on-boad LED
			GPIO_ConfigLEDPA5(GPIO_Reset);
		}

	}
}
