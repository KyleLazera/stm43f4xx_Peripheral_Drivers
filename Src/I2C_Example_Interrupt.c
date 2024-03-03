#include "stm32f401_i2c.h"

/*
 * The I2C driver was tested on the DS1307 Real time clock module. This example file is not a driver for the module,
 * but rather, a very simplistic program meant to test the I2C driver by transmitting data to the
 * module and then reading it and placing it into an empty array.
 *
 * This specific example uses non-blocking commmunication, which allows the CPU to continue processsing data while
 * communication is underway. This is achieved utilizing interrupts and an interrupt event handler. Due to this however,
 * I added time delays in between each initiation of a new transmission/reception. This prevents the CPU from moving
 * to the next transmission/reception prior to completing the previous one.
 */

const uint8_t slave_address = 0b1101000;

I2C_Handle_t I2C_Example;
UART_Config_t UART2;

void I2C1_EV_IRQHandler();
void delay(int time_delay);

int main()
{

	//This data represents: 06:47:00 AM on 01/14/2023 with the clock kept in 24 hour mode
	uint8_t output_data[8] = {0x00, 0, 71, 6, 7, 20, 1, 35};
	uint8_t output_register_address[1] = {0x00};
	uint8_t input_data[7];

	//Configure UART settings
	UART_Config(&UART2, USART2, UART_MODE_TX, 115200);

	//Initilize the UART
	UART_Init(&UART2);

	I2C_Config(&I2C_Example, I2C1, SM_100KHZ, FM_DUTY_2, Pin8, Pin9);

	I2C_Init(&I2C_Example);

	//Begin the data transmission using an interrupt
	I2C_MasterTransmitIT(&I2C_Example, output_data, slave_address, 8);

	//Delay the CPU to wait for the interrupt handler to complete
	delay(100000);

	//Begin a second data transmission to retun the register pointer back to 0x0
	I2C_MasterTransmitIT(&I2C_Example, output_register_address, slave_address, 1);

	//Teporary delay
	delay(100000);

	//Read data from the slave device
	I2C_MasterRecieveIT(&I2C_Example, input_data, slave_address, 7);

	//Delay once again to prevent CPU from running through rest of code while communication is still ongoing
	delay(100000);

	for(int i = 0; i < 8; i++)
	{
		//For purpose of debugging, I chose to print each byte individually as opposed to printing the array all at once
		WriteByte(&UART2, input_data[i]);
	}

	//Deactivate the I2C Bus
	I2C_DeInit(&I2C_Example);

	while(1)
	{

	}
}

//Event handler
void I2C1_EV_IRQHandler()
{
	IRQ_Event_Handler(&I2C_Example);
}

//Temporary time delay - the input is not indicative of the amount of time it will actualy take
void delay(int time_delay)
{
	for(int i = 0; i < time_delay; i++){}
}


