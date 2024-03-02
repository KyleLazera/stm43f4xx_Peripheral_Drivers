//#include "stm32f401_spi.h"

/*
 * This displays a simple SPI non-blocking transmission with the ADXL345 and using the multi-slave mode. To adequeatly use the
 * multislave mode with interrupts, the GPIO driver included in the drivers file must be used. This allows the user the ability
 * to create multiple slaves and choose which slave to communicate to based off the specified slave. For this example, I did not include
 * a second slave purely because it is being demo'd on a breadboard, which introduces a lot of noise. This example is purely to display how to
 * to use the SPI in this mode.
 *

SPI_Handle_t SPI1_Example;
GPIO_Config_t Slave1, Slave2;

/*
 * Uncomment code below and comment the equivelent variables in the main() out to use the
 * live expressions debuggin feature.
 *
int16_t x, y, z;
double xg, yg, zg;
uint8_t adxl_data_rec[7];

void SPI_Specs_Init();
void SPI1_IRQHandler();

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

	//Create and initialze a slave pin with the desired port and pin
	GPIO_Config(&Slave1, GPIOA, Pin8, GPIO_Output, GPIO_PushPull, GPIO_LowSpeed, GPIO_PullUp);
	GPIO_Init(&Slave1, 0x0);
	GPIOA->ODR |= (1U << 8); //Used to enable the P-MOS and ensure CS is active high

	//GPIO_Config(&Slave2, GPIOA, Pin9, GPIO_Output, GPIO_PushPull, GPIO_LowSpeed, GPIO_PullUp);
	//GPIO_Init(&Slave2, 0x0);
	//GPIOA->ODR |= (1U << 9); //Used to enable the P-MOS and ensure CS is active high

	SPI_Specs_Init();

	SPI_Init(&SPI1_Example);

	/*
	 * Initial Transmission to send to the ADXL. Sets the ADXL to specified settings.
	 *
	SPI_MultiSlave_TransmitIT(&SPI1_Example, &Slave1, adxl_clear_powerctl_reg, 2);
	SPI_MultiSlave_TransmitIT(&SPI1_Example, &Slave1, adxl_set_data_format, 2);
	SPI_MultiSlave_TransmitIT(&SPI1_Example, &Slave1, daxl_set_bw_rate_reg, 2);
	SPI_MultiSlave_TransmitIT(&SPI1_Example, &Slave1, adxl_set_powerctl_reg, 2);

	//SPI_MultiSlave_TransmitIT(&SPI1_Example, &Slave2, adxl_set_powerctl_reg, 2);
	//SPI_MultiSlave_TransmitIT(&SPI1_Example, &Slave2, adxl_set_powerctl_reg, 2);
	//SPI_MultiSlave_TransmitIT(&SPI1_Example, &Slave2, adxl_set_powerctl_reg, 2);

	while(1)
	{
		//Reading data from the SPI
		SPI_MultiSlave_RecieveIT(&SPI1_Example, &Slave1, adxl_data_rec, 7, *adxl_address);

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
	SPI1_Example.SPI_Config.pin_miso = Pin6;
	SPI1_Example.SPI_Config.pin_mosi = Pin7;

	//Set desired transfer methods
	SPI1_Example.ssm = SSM_Enable;
	SPI1_Example.SPIx = SPI1;
	SPI1_Example.SPI_Config.baudrate_ctrl = DIV4;
	SPI1_Example.SPI_Config.cpha = Falling_Edge;
	SPI1_Example.SPI_Config.cpol = Even_Polarity;
	SPI1_Example.SPI_Config.data_format = MSB_First;
	SPI1_Example.data_frame = Data_8_Bits;
}*/
