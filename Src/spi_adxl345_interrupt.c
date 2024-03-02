//#include "stm32f401_spi.h"

/*
 * This program displays a simple SPI, non-blocking function using the built in slave select with the MCU. To use this
 * set the specified pin in the SPI_Specs function. I find this feature to introduce a lot more noise into the data
 * specifically when compared to the non-blocking multi-slave example.
 *

SPI_Handle_t SPI1_Example;
GPIO_Config_t Slave1;

/*
 * Uncomment code below and comment the equivelent variables in the main() out to use the
 * live expressions debuggin feature.
 *
int16_t x, y, z;
double xg, yg, zg;
uint8_t adxl_data_rec[7];

void SPI_Specs_Init();

int main()
{
	/*
	 * Data that holds the address of the data registers of the ADXL - this is where data will be read
	 * from (Not the address of the device). A buffer that will hold the data being read.
	 *
	uint8_t adxl_address[1] = {0xF2};


	/*
	 * Functions to initialize the ADXL registers to set range to +- 4g, to allow
	 * continous reading of the data registers and set the transfer rate to 100Hz.
	 *
	uint8_t adxl_set_data_format[2] = {0x31, 0x01};
	uint8_t adxl_clear_powerctl_reg[2] = {0x2D, 0x00};
	uint8_t adxl_set_powerctl_reg[2] = {0x2D, 0x08};
	uint8_t daxl_set_bw_rate_reg[2] = {0x2C, 0x0A};

	SPI_Specs_Init();

	SPI_Init(&SPI1_Example);

	/*
	 * Initial Transmission to send to the ADXL. Sets the ADXL to specified settings.
	 *
	SPI_TransmitIT(&SPI1_Example, adxl_clear_powerctl_reg, 2);
	SPI_TransmitIT(&SPI1_Example, adxl_set_data_format, 2);
	SPI_TransmitIT(&SPI1_Example, daxl_set_bw_rate_reg, 2);
	SPI_TransmitIT(&SPI1_Example, adxl_set_powerctl_reg, 2);


	while(1)
	{
		SPI_ReceiveIT(&SPI1_Example, adxl_data_rec, 7, *adxl_address);

		x = ((adxl_data_rec[2] << 8) | adxl_data_rec[1]);
		y = ((adxl_data_rec[4] << 8) | adxl_data_rec[3]);
		z = ((adxl_data_rec[6] << 8) | adxl_data_rec[5]);

		xg = (x * 0.0078);
		yg = (y * 0.0078);
		zg = (z * 0.0078);
	}
}

/*
 * Function to serve interrupts
 *
void SPI1_IRQHandler()
{
	SPI_IRQ_Handler(&SPI1_Example);
}

/*
 * Function to set the SPI specs
 *
void SPI_Specs_Init()
{
	//Set desired pins
	SPI1_Example.SPI_Config.cs_gpio = GPIOA;
	SPI1_Example.SPI_Config.pin_clk = Pin5;
	SPI1_Example.SPI_Config.pin_cs = Pin4;
	SPI1_Example.SPI_Config.pin_miso = Pin6;
	SPI1_Example.SPI_Config.pin_mosi = Pin7;

	//Set desired transfer methods
	SPI1_Example.ssm = SSM_Disable;
	SPI1_Example.SPIx = SPI1;
	SPI1_Example.SPI_Config.baudrate_ctrl = DIV4;
	SPI1_Example.SPI_Config.cpha = Falling_Edge;
	SPI1_Example.SPI_Config.cpol = Even_Polarity;
	SPI1_Example.SPI_Config.data_format = MSB_First;
	SPI1_Example.data_frame = Data_8_Bits;
}*/

