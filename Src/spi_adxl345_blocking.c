#include "stm32f401_spi.h"

/*
 * This is a basic SPI blocking function that utilizes the built in slave select feature provided by the MCU. To change this feature to enable
 * multiple slaves, change ss_disable to SSM_Enable and enable a GPIO pin as output for the slave select. To enable this function, you would
 * also have to manually pull the GPIO pin high and low depending on what the desired output it. To see an example of using multislave device,
 * see the adxl345 multislave interrupt example.
 */

SPI_Handle_t SPI1_Example;

/*
 * The variables below hold the data read in and are made global purely to use the live expression debugger,
 * and keep track of the variable values.
 */
int16_t x, y, z;
double xg, yg, zg;
uint8_t adxl_data_rec[6];

void SPI_Specs_Init();

int main()
{
	/*
	 * Data that holds the address of the data registers of the ADXL - this is where data will be read
	 * from (Not the address of the device). A buffer that will hold the data being read.
	 */
	uint8_t adxl_address[1] = {0xF2};

	/*
	 * Functions to initialize the ADXL registers to set range to +- 4g, to allow
	 * continous reading of the data registers and set the transfer rate to 100Hz.
	 */
	uint8_t adxl_set_data_format[2] = {0x31, 0x01};
	uint8_t adxl_clear_powerctl_reg[2] = {0x2D, 0x00};
	uint8_t adxl_set_powerctl_reg[2] = {0x2D, 0x08};
	uint8_t daxl_set_bw_rate_reg[2] = {0x2C, 0x0A};

	SPI_Specs_Init();

	SPI_Init(&SPI1_Example);

	/*
	 * Initial Transmission to send to the ADXL. Sets the ADXL to specified settings.
	 */
	SPI_Transmit(&SPI1_Example, adxl_clear_powerctl_reg, 2, No_Restart);
	SPI_Transmit(&SPI1_Example, adxl_set_data_format, 2, No_Restart);
	SPI_Transmit(&SPI1_Example, daxl_set_bw_rate_reg, 2, No_Restart);
	SPI_Transmit(&SPI1_Example, adxl_set_powerctl_reg, 2, No_Restart);


	while(1)
	{
		//Reading data from the SPI
		SPI_Transmit(&SPI1_Example, adxl_address, 1, Restart);
		SPI_Receive(&SPI1_Example, adxl_data_rec, 7);

		//Processing the data
		x = ((adxl_data_rec[1] << 8) | adxl_data_rec[0]);
		y = ((adxl_data_rec[3] << 8) | adxl_data_rec[2]);
		z = ((adxl_data_rec[5] << 8) | adxl_data_rec[4]);

		xg = (x * 0.0078);
		yg = (y * 0.0078);
		zg = (z * 0.0078);
	}
}

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
}


