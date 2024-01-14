/*
 * The I2C driver was tested on the DS1307 Real time clock module. This example file is not a driver for the module,
 * but rather, a very simplistic program meant to test the I2C driver by transmitting data to the
 * module and then reading it and placing it into an empty array.
 *
 * This specific example uses blocking commmunication, which blocks the CPU from running other programs while
 *it is communicating with the slave. This is because while loops are used to monitor the state of the I2C flags.
 */

#include "stm32f401_i2c.h"

const uint8_t slave_address = 0b1101000;

I2C_Handle_t I2C_Example;
UART_Config_t UART2;

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

	//Configure the I2C clock settings
	I2C_Config(&I2C_Example, I2C1, SM_100KHZ, FM_DUTY_2, Pin8, Pin9);

	//Initialize I2C
	I2C_Init(&I2C_Example);

	//Transmit data into the real time clock
	I2C_MasterTransmit(&I2C_Example, output_data, slave_address, 8, I2C_Restart);

	//Transmit 0x0 to return the address pointer to the top of the stack in the slave
	I2C_MasterTransmit(&I2C_Example, output_register_address, slave_address, 1, I2C_Restart);

	//Read the data from the slave
	I2C_MasterRecieve(&I2C_Example, input_data, slave_address, 7, I2C_No_Restart);

	for(int i = 0; i < 8; i++)
	{
		//For purpose of debugging, I chose to print each byte individually as opposed to printing the array all at once
		WriteByte(&UART2, input_data[i]);
	}

	//Deactivate I2C access
	I2C_DeInit(&I2C_Example);


	while(1)
	{

	}
}






