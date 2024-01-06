#include "stm32f401_i2c.h"

I2C_Handle_t I2C_Example;
UART_Config_t UART2;

void I2C1_EV_IRQHandler();

const uint8_t slave_address = 0b1101000;

int main()
{
	uint8_t output_data[8] = {0x00, 5, 14, 70, 2, 8, 18, 36};
	uint8_t output_register_address[1] = {0x00};
	uint8_t input_data[7];

	//Configure UART settings
	UART_Config(&UART2, USART2, UART_MODE_TX, 115200);

	//Initilize the UART
	UART_Init(&UART2);

	I2C_Config(&I2C_Example, I2C1, SM_100KHZ, FM_DUTY_2, Pin8, Pin9);

	I2C_DeInit(&I2C_Example);

	I2C_Init(&I2C_Example);

	I2C_MasterTransmit(&I2C_Example, output_data, slave_address, 7, I2C_Restart);
	//I2C_MasterTransmitIT(&I2C_Example, output_data, slave_address, 7, I2C_No_Restart);

	I2C_MasterTransmit(&I2C_Example, output_register_address, slave_address, 1, I2C_Restart);
	//I2C_MasterTransmitIT(&I2C_Example, output_register_address, slave_address, 1, I2C_No_Restart);

	I2C_MasterRecieve(&I2C_Example, input_data, slave_address, 7, I2C_No_Restart, &UART2);
	//I2C_MasterRecieveIT(&I2C_Example, input_data, slave_address, 7, I2C_No_Restart);

	/*for(int i = 0; i < 7; i++)
	{
		//For purpose of debugging, I chose to print each byte individually as opposed to printing the array all at once
		//PrintData(&UART2, "\n\rData: \n\r\n\r");
		WriteByte(&UART2, input_data[i]);
	}*/


	while(1)
	{

	}
}

void I2C1_EV_IRQHandler()
{
	IRQ_Event_Handler(&I2C_Example, &UART2);
}




