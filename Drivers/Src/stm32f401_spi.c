#include "stm32f401_spi.h"
#include <math.h>

/*
 * @brief	Helper function used to enable the SPI GPIO pins
 */
static void Config_SPI_Periph(GPIO_TypeDef *GPIOx, uint8_t pin, AFR_Config_t alt_function)
{
	GPIO_Config_t SPI_Periph;
	GPIO_Config(&SPI_Periph, GPIOx, pin, GPIO_AF, GPIO_PushPull, GPIO_LowSpeed, GPIO_PUPD_None);
	GPIO_Init(&SPI_Periph, alt_function);
}

/*
 * @brief	Helper function to initialize the SPI peripheral based on specified pins
 *
 * @note	Sets the specific port and alternate function for the specified SPI
 */
static void Enable_SPI_Periph(SPI_Handle_t *SPI_Handle)
{
	//Initialize varibales for pins
	uint8_t nss_pin, sck_pin, mosi_pin, miso_pin;
	nss_pin = SPI_Handle->SPI_Config.pin_nss;
	sck_pin = SPI_Handle->SPI_Config.pin_sck;
	mosi_pin = SPI_Handle->SPI_Config.pin_mosi;
	miso_pin = SPI_Handle->SPI_Config.pin_miso;

	if(SPI_Handle->SPIx == SPI1)
	{
		//Check is ssm is set - if not, enable the specified nss pin
		if(!SPI_Handle->SPI_Config.ssm)
		{
			//Init slave select pin
			Config_SPI_Periph(GPIOA, nss_pin, AF5);
		}

		//Init serial clock
		switch(sck_pin)
		{
		case Pin5:
			Config_SPI_Periph(GPIOA, sck_pin, AF5);
			break;
		case Pin3:
			Config_SPI_Periph(GPIOB, sck_pin, AF5);
			break;
		}

		//Init mosi pin
		switch(miso_pin)
		{
		case Pin6:
			Config_SPI_Periph(GPIOA, mosi_pin, AF5);
			break;
		case Pin4:
			Config_SPI_Periph(GPIOB, mosi_pin, AF5);
			break;

		}

		//Init miso pin
		switch(mosi_pin)
		{
		case Pin7:
			Config_SPI_Periph(GPIOA, miso_pin, AF5);
			break;
		case Pin5:
			Config_SPI_Periph(GPIOB, miso_pin, AF5);
			break;
		}

		//Enable Clock Access to SPI1
		RCC_APB2Cmd(SPI1_Enable, ENABLE);
	}

	else if(SPI_Handle->SPIx == SPI2)
	{
		if(SPI_Handle->SPI_Config.ssm)
		{
			//Init slave select pin
			Config_SPI_Periph(GPIOB, nss_pin, AF5);
		}

		//Init serial clock pin
		switch(sck_pin)
		{
		case Pin3:
			Config_SPI_Periph(GPIOD, sck_pin, AF5);
			break;
		default:
			Config_SPI_Periph(GPIOB, sck_pin, AF5);
			break;
		}

		//Init mosi pin
		switch(mosi_pin)
		{
		case Pin3:
			Config_SPI_Periph(GPIOC, mosi_pin, AF5);
			break;
		case Pin15:
			Config_SPI_Periph(GPIOB, mosi_pin, AF5);
			break;
		}

		//Init miso pin
		switch(miso_pin)
		{
		case Pin2:
			Config_SPI_Periph(GPIOC, miso_pin, AF5);
			break;
		case Pin14:
			Config_SPI_Periph(GPIOB, miso_pin, AF5);
			break;
		}

		//Enable Clock Access to SPI2
		RCC_APB1Cmd(SPI2_Enable, ENABLE);
	}

	else if(SPI_Handle->SPIx == SPI3)
	{
		if(SPI_Handle->SPI_Config.ssm)
		{
			//Init slave select pin
			Config_SPI_Periph(GPIOA, nss_pin, AF6);
		}

		//Init serial clock pin
		switch(sck_pin)
		{
		case Pin3:
			Config_SPI_Periph(GPIOB, sck_pin, AF6);
			break;
		case Pin10:
			Config_SPI_Periph(GPIOC, sck_pin, AF6);
			break;
		}

		//Init mosi pin
		switch(mosi_pin)
		{
		case Pin12:
			Config_SPI_Periph(GPIOC, mosi_pin, AF6);
			break;
		case Pin6:
			Config_SPI_Periph(GPIOD, mosi_pin, AF5);
			break;
		case Pin5:
			Config_SPI_Periph(GPIOB, mosi_pin, AF6);
			break;
		}

		//Init miso pin
		switch(miso_pin)
		{
		case Pin11:
			Config_SPI_Periph(GPIOC, miso_pin, AF6);
			break;
		case Pin4:
			Config_SPI_Periph(GPIOB, miso_pin, AF6);
			break;
		}
		//Enable Clock Access to SPI3
		RCC_APB1Cmd(SPI3_Enable, ENABLE);
	}

	else
	{
		if(SPI_Handle->SPI_Config.ssm)
		{
			//Init slave select pin
			Config_SPI_Periph(GPIOE, nss_pin, AF5);
		}
		Config_SPI_Periph(GPIOE, sck_pin, AF5);
		Config_SPI_Periph(GPIOE, miso_pin, AF5);
		Config_SPI_Periph(GPIOE, mosi_pin, AF5);

		//Enable Clock Access to SPI4
		RCC_APB2Cmd(SPI4_Enable, ENABLE);
	}
}

/*
 * @brief	Initialize the SPI peripheral with the user specified information.
 *
 * @note	This function Allows GPIO access to the specified pins, enables clock access,
 * 			configures the SSM and SSOE bit, calculates the clock divisor and sets other
 * 			specified values such as the CPHA, CPOL, MSTR and more.
 *
 * @note	For the clock rate, the user specifies the desired clock speed and an algorithm
 * 			determines what the APB1/APB2 bus needs to be divided by to achieve that speed.
 * 			To achieve the desired speed, the user needs to ensure their APB1/APB2 speed is divisibe
 * 			by 2^n, where n ranges from 1-8 inclusive.
 *
 * @note	The users peripheral clock speed can be adjusted based off the RCC driver functions to achieve
 * 			the required frequency.
 *
 * @note	If SSM bit is set, the SSI bit will be controled by software and will determine which slave has been
 * 			selected. The soecified nss GPIO pin, will not be activated. If SSM = 0, the GPIO pins will be active,
 * 			and the SSOE bit will need to be set to determine whether the user wants the MCU to act as a master or have
 * 			multimaster capability.
 */
void SPI_Init(SPI_Handle_t *SPI_Handle)
{
	RCC_ClockFrequency_t ClockSource;

	uint32_t clock_rate = 0;
	//Used to keep track of the bit value to input into the CR1 register
	uint8_t bit_value = 0;

	//Enable specified pins and clock access to specified SPI
	Enable_SPI_Periph(SPI_Handle);

	//Check if software slave management is enabled
	if(SPI_Handle->SPI_Config.ssm)
	{
		//Enable ssm bit in CR1
		SPI_Handle->SPIx->CR1 |= CR1_SSM_Enable;
	}

	else
	{
		//if SSM is disabled check for SSOE flag
		if(SPI_Handle->SPI_Config.ssoe)
		{
			//Enable SSOE flag
			SPI_Handle->SPIx->CR2 |= CR2_SSOE_Enable;
		}
	}

	//Determine Peripheral Clock Speed
	RCC_GetClockFreq(&ClockSource);

	//Alorithm to determine the divisor based off the clock speed input by the user
	while(clock_rate != SPI_Handle->SPI_Config.clock_rate)
	{
		bit_value++;

		//Check for which SPI peripheral is enabled - this determines whether to use APB1 or APB2
		if(SPI_Handle->SPIx == SPI1 || SPI_Handle->SPIx == SPI4)
		{
			clock_rate = (ClockSource.PCLCK1) >> (bit_value);
		}
		else
		{
			clock_rate = (ClockSource.PCLCK2) >> (bit_value);
		}

		//Ensure the while loop does not get caught in an infinte loop
		if(bit_value > 8)
		{
			return;
		}
	}

	//Set Baud rate control
	SPI_Handle->SPIx->CR1 |= ((bit_value - 1) << CR1_BR_Pos);

	//Set CPOL and CPHA
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.cpol) << CR1_CPOL_Pos);
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.cpha) << CR1_CPHA_Pos);

	//Set to master or slave
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.spi_bus_direction) << CR1_MSTR_Pos);

	//Set the data frame & LSB first
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.lsbfirst) << CR1_LSBFIRST_Pos);
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.data_frame) << CR1_DFF_Pos);

	//Enable SPI peripheral
	SPI_Handle->SPIx->CR1 |= CR1_SPE_Enable;
}

