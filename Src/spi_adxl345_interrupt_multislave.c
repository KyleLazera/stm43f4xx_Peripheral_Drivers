#include "stm32f401_spi.h"

/*
 * This displays a simple SPI non-blocking transmission with the ADXL345 and using the multi-slave mode. To adequeatly use the
 * multislave mode with interrupts, the GPIO driver included in the drivers file must be used. This allows the user the ability
 * to create multiple slaves and choose which slave to communicate to based off the specified slave. For this example, I included the adxl345
 * along with the bme280 pressure sensor. Both of these slaves communicate with the mcu by adjusting the specified slave passed to the functions.
 * In this specific example, every time the accelerometer is read 10000 times, the bme sensor reads a value.
 */

SPI_Handle_t SPI1_Example;
GPIO_Config_t ADXL, BME;

/*
 * The data below is made global to allow for use of the live expression debugger. This allows the user to see the data real time
 * in the case they do not have an oscilliscope/logic analyzer. Additionally, this lets the user see the calculated values.
 */
int16_t x, y, z;
double xg, yg, zg;
uint8_t adxl_data_rec[7];
uint8_t	bme_data_rec[9];

void SPI_Specs_Init();
void SPI1_IRQHandler();

int main()
{
	int counter = 0;

	/*
	 * Data that holds the address of the data registers of the ADXL - this is where data will be read
	 * from (Not the address of the device). A buffer that will hold the data being read.
	 */
	uint8_t adxl_address[1] = {0xF2};
	uint8_t bme_address[1] = {0xF7};

	/*
	 * Variables to initialize the ADXL registers to set range to +- 4g, to allow
	 * continous reading of the data registers and set the transfer rate to 100Hz.
	 */
	uint8_t adxl_set_data_format[2] = {0x31, 0x01};
	uint8_t adxl_clear_powerctl_reg[2] = {0x2D, 0x00};
	uint8_t adxl_set_powerctl_reg[2] = {0x2D, 0x08};
	uint8_t daxl_set_bw_rate_reg[2] = {0x2C, 0x0A};

	/*
	 * Variables to initialize the BME280 registers to enable pressure, temperature and humidity readings.
	 * Unlike with the ADXL345, the registers are contigious therefore I am burst writing. Additionally used to
	 * force a measurement.
	 */
	uint8_t bme_enable_registers[6] = {0x72, 0x01, 0x74, 0x24, 0x75, 0x80};
	uint8_t bme_force_measure[2] = {0x74, 0x25};

	//Create and initialze a slave pin with the desired port and pin
	GPIO_Config(&ADXL, GPIOA, Pin8, GPIO_Output, GPIO_PushPull, GPIO_LowSpeed, GPIO_PullUp);
	GPIO_Init(&ADXL, 0x0);
	GPIOA->ODR |= (1U << 8); //Used to enable the P-MOS and ensure CS is active high

	GPIO_Config(&BME, GPIOA, Pin9, GPIO_Output, GPIO_PushPull, GPIO_LowSpeed, GPIO_PullUp);
	GPIO_Init(&BME, 0x0);
	GPIOA->ODR |= (1U << 9); //Used to enable the P-MOS and ensure CS is active high

	SPI_Specs_Init();

	SPI_Init(&SPI1_Example);

	/*
	 * Initial Transmission to send to the ADXL. Sets the ADXL to specified settings.
	 */
	SPI_MultiSlave_TransmitIT(&SPI1_Example, &ADXL, adxl_clear_powerctl_reg, 2);
	SPI_MultiSlave_TransmitIT(&SPI1_Example, &ADXL, adxl_set_data_format, 2);
	SPI_MultiSlave_TransmitIT(&SPI1_Example, &ADXL, daxl_set_bw_rate_reg, 2);
	SPI_MultiSlave_TransmitIT(&SPI1_Example, &ADXL, adxl_set_powerctl_reg, 2);

	SPI_MultiSlave_TransmitIT(&SPI1_Example, &BME, bme_enable_registers, 2);

	while(1)
	{
		if(counter % 10000 == 0){
			SPI_MultiSlave_TransmitIT(&SPI1_Example, &BME, bme_force_measure, 2);
			SPI_MultiSlave_RecieveIT(&SPI1_Example, &BME, bme_data_rec, 8, *bme_address);
		}

		//Reading data from the SPI
		SPI_MultiSlave_RecieveIT(&SPI1_Example, &ADXL, adxl_data_rec, 7, *adxl_address);

		x = ((adxl_data_rec[2] << 8) | adxl_data_rec[1]);
		y = ((adxl_data_rec[4] << 8) | adxl_data_rec[3]);
		z = ((adxl_data_rec[6] << 8) | adxl_data_rec[5]);

		xg = (x * 0.0078);
		yg = (y * 0.0078);
		zg = (z * 0.0078);

		counter++;
	}
}

/*
 * Function to serve interrupts
 */
void SPI1_IRQHandler()
{
	SPI_IRQ_Handler(&SPI1_Example);
}

/*
 * Function to set the SPI specs
 */
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
}
