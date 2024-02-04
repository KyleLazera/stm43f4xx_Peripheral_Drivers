#include "stm32f401_spi.h"


int main()
{
	SPI_Handle_t SPI_Example;
	UART_Config_t Debug;

	uint8_t data_transmit[4] = {0x74, 0x40, 0x75, 0x80};
	uint8_t data_recieve[5];

	UART_Config(&Debug, USART2, UART_MODE_TX, 115200);
	UART_Init(&Debug);

	SPI_Example.SPI_Config.ssm = SSM_Disable;
	SPI_Example.SPI_Config.ssoe = SSOE_Enable;
	SPI_Example.SPIx = SPI1;
	SPI_Example.SPI_Config.pin_sck = Pin5;
	SPI_Example.SPI_Config.pin_nss = Pin4;
	SPI_Example.SPI_Config.pin_mosi = Pin7;
	SPI_Example.SPI_Config.pin_miso = Pin6;
	SPI_Example.SPI_Config.clock_divisor = DIV16; //This will set SPI Clk to 1MHz (16MHz/16)

	SPI_Example.SPI_Config.data_frame = Data_8_Bits;
	SPI_Example.SPI_Config.cpha = Falling_Edge;
	SPI_Example.SPI_Config.cpol = Even_Polarity;
	SPI_Example.SPI_Config.lsbfirst = MSB_First;
	SPI_Example.SPI_Config.spi_bus_direction = SPI_Master;

	SPI_Init(&SPI_Example);

	SPI_Transmit(&SPI_Example, data_transmit, 4);

	SPI_Recieve(&SPI_Example, data_recieve, 3, 0xF4);

	for(int i = 0; i < 3; i++)
	{
		WriteByte(&Debug, data_recieve[i]);
	}


	while(1)
	{

	}
}


