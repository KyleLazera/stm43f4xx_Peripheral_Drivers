#include "stm32f401_spi.h"

int main()
{
	SPI_Handle_t SPI_Example;

	SPI_Example.SPI_Config.ssm = SSM_Enable;
	SPI_Example.SPIx = SPI1;
	SPI_Example.SPI_Config.pin_sck = Pin5;
	SPI_Example.SPI_Config.pin_nss = Pin4;
	SPI_Example.SPI_Config.pin_mosi = Pin7;
	SPI_Example.SPI_Config.pin_miso = Pin6;
	SPI_Example.SPI_Config.clock_rate = 5756;

	SPI_Example.SPI_Config.data_frame = Data_8_Bits;
	SPI_Example.SPI_Config.cpha = Rising_Edge;
	SPI_Example.SPI_Config.cpol = Odd_Polarity;
	SPI_Example.SPI_Config.lsbfirst = LSB_First;
	SPI_Example.SPI_Config.spi_bus_direction = SPI_Master;

	RCC_PCLCK1Config(RCC_APB1Prescaler_2);

	SPI_Init(&SPI_Example);

	while(1)
	{

	}
}
