#include "stm32f401_spi.h"

int16_t x, y, z;
double xg, yg, zg;


uint8_t adxl_data_rec[7];
uint8_t bme_data_rec[9];
uint8_t bme_address[1] = {0xF7};
uint8_t adxl_address[1] = {0xF2};
uint8_t adxl_address1 = 0xF2;
int work_done = 0;

SPI_Handle_t SPI1_Example;
UART_Config_t Debug;


void cs_disable(uint8_t slave_number);
void cs_enable(uint8_t slave_number);

int main()
{
	uint8_t bme_init[6] = {0x72, 0x01, 0x74, 0x27, 0x75, 0x80};
	uint8_t adxl_init1[2] = {0x31, 0x01};
	uint8_t adxl_init2[2] = {0x2D, 0x00};
	uint8_t adxl_init3[2] = {0x2D, 0x08};
	uint8_t adxl_init4[2] = {0x2C, 0x0A}; //Default is 0x0A for 100Hz output

	UART_Config(&Debug, USART2, UART_MODE_TX, 115200);
	UART_Init(&Debug);

	SPI1_Example.SPI_Config.cs_gpio = GPIOA;
	SPI1_Example.SPI_Config.pin_clk = Pin5;
	SPI1_Example.SPI_Config.pin_cs = Pin4; //Used for SSM disabled
	//SPI1_Example.SPI_Config.pin_cs = Pin9; //Used for ADXL345 slave
	SPI1_Example.SPI_Config.pin_miso = Pin6;
	SPI1_Example.SPI_Config.pin_mosi = Pin7;

	SPI1_Example.ssm = SSM_Disable;
	SPI1_Example.SPIx = SPI1;
	SPI1_Example.SPI_Config.baudrate_ctrl = DIV4;
	SPI1_Example.SPI_Config.cpha = Falling_Edge;
	SPI1_Example.SPI_Config.cpol = Even_Polarity;
	SPI1_Example.SPI_Config.data_format = MSB_First;
	SPI1_Example.data_frame = Data_8_Bits;

	SPI_Init(&SPI1_Example);

	//cs_enable(0);
	//SPI_Transmit(&SPI1_Example, adxl_init2, 2, No_Restart);
	SPI_TransmitIT(&SPI1_Example, adxl_init2, 2);
	//cs_disable(0);

	//cs_enable(0);
	//SPI_Transmit(&SPI1_Example, adxl_init1, 2, No_Restart);
	SPI_TransmitIT(&SPI1_Example, adxl_init1, 2);
	//cs_disable(0);

	//cs_enable(0);
	//SPI_Transmit(&SPI1_Example, adxl_init4, 2, No_Restart);
	SPI_TransmitIT(&SPI1_Example, adxl_init4, 2);
	//cs_disable(0);

	//cs_enable(0);
	//SPI_Transmit(&SPI1_Example, adxl_init3, 2, No_Restart);
	SPI_TransmitIT(&SPI1_Example, adxl_init3, 2);
	//cs_disable(0);


	//cs_enable(0);
	//SPI_Transmit(&SPI1_Example, bme_init, 6, Restart);
	//cs_disable(0);

	while(1)
	{
		//cs_enable(0);
		//SPI_Transmit(&SPI1_Example, adxl_address, 1, Restart);
		//SPI_Receive(&SPI1_Example, adxl_data_rec, 6);
		//cs_disable(0);
		SPI_ReceiveIT(&SPI1_Example, adxl_data_rec, 7, *adxl_address);


		x = ((adxl_data_rec[2] << 8) | adxl_data_rec[1]);
		y = ((adxl_data_rec[4] << 8) | adxl_data_rec[3]);
		z = ((adxl_data_rec[6] << 8) | adxl_data_rec[5]);

		//x = ((adxl_data_rec[1] << 8) | adxl_data_rec[0]);
		//y = ((adxl_data_rec[3] << 8) | adxl_data_rec[2]);
		//z = ((adxl_data_rec[5] << 8) | adxl_data_rec[4]);

		xg = (x * 0.0078);
		yg = (y * 0.0078);
		zg = (z * 0.0078);

		work_done++;
	}
}

void SPI1_IRQHandler()
{
	SPI_IRQ_Handler(&SPI1_Example);
}

void cs_enable(uint8_t slave_number)
{
	if(slave_number == 1)
	{
		GPIOA->ODR &= ~(1U << 8);
	}
	else
	{
		GPIOA->ODR &= ~(1U << 9);
	}
}

void cs_disable(uint8_t slave_number)
{
	if(slave_number == 1)
	{
		GPIOA->ODR |= (1U << 8);
	}
	else
	{
		GPIOA->ODR |= (1U << 9);
	}
}


