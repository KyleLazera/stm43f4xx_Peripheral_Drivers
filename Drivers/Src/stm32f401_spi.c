#include "stm32f401_spi.h"

/*
 * brief	A helper function that keeps track of specific flags in the status register.
 *
 * @retval	Returns status of the flag which is defined in an enumeration.
 */
static Flag_Status Check_Flag(SPI_Handle_t *SPI_Handle, uint32_t flag)
{
	if(SPI_Handle->SPIx->SR & flag)
	{
		return Flag_Set;
	}

	else
	{
		return Flag_Unset;
	}
}

/*
 * @brief	This is a helper function that disables the SPI peripheral after transferring data
 */
static void Disable_SPI(SPI_Handle_t *SPI_Handle)
{
		//Ensure the TXE is raised and BSY flag = 0
		while(!(Check_Flag(SPI_Handle, SR_TXE_Flag)));
		while(Check_Flag(SPI_Handle, SR_BSY_Flag));
		//Disable the SPI peripheral
		SPI_Handle->SPIx->CR1 &= ~CR1_SPE_Enable;
}

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
			SPI_Handle->GPIOx = GPIOA;
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
			SPI_Handle->GPIOx = GPIOB;
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
			SPI_Handle->GPIOx = GPIOA;
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
			SPI_Handle->GPIOx = GPIOE;
		}
		Config_SPI_Periph(GPIOE, sck_pin, AF5);
		Config_SPI_Periph(GPIOE, miso_pin, AF5);
		Config_SPI_Periph(GPIOE, mosi_pin, AF5);

		//Enable Clock Access to SPI4
		RCC_APB2Cmd(SPI4_Enable, ENABLE);
	}
}

/*
 * @brief	Configuration function that allows the user to specify the pins for the SPI
 */
void SPI_Pin_Config(SPI_Handle_t *SPI_Handle, uint8_t clock, uint8_t mosi, uint8_t miso, uint8_t nss)
{
	SPI_Handle->SPI_Config.pin_sck = clock;
	SPI_Handle->SPI_Config.pin_mosi = mosi;
	SPI_Handle->SPI_Config.pin_miso = miso;
	SPI_Handle->SPI_Config.pin_nss = nss;
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

	//Set Baud rate control
	SPI_Handle->SPIx->CR1 |= (SPI_Handle->SPI_Config.clock_divisor << CR1_BR_Pos);

	//Set CPOL and CPHA
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.cpol) << CR1_CPOL_Pos);
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.cpha) << CR1_CPHA_Pos);

	//Set to master or slave
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.spi_bus_direction) << CR1_MSTR_Pos);

	//Set the data frame & LSB first
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.lsbfirst) << CR1_LSBFIRST_Pos);
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.data_frame) << CR1_DFF_Pos);

}

/*
 * @brief	This function is used to transmit data on the MOSI pin.
 *
 * @note	This function can also be used by setting BIDIMODE = 1 and BIDIOE = 1, which enables
 * 			bidirection transmission only. This would remove the chances of a value accidentally being
 * 			written on the MISO line from the slave.
 *
 * @param	pTxBuffer: Holds an address for the array/pointer that holds the data to be transmitted.
 *
 * @param	num_of_bytes: Specifies the total number of bytes to be transmitted.
 */
void SPI_Transmit(SPI_Handle_t *SPI_Handle, uint8_t *pTxBuffer, uint8_t num_of_bytes)
{
	//Enable SPI peripheral
	SPI_Handle->SPIx->CR1 |= CR1_SPE_Enable;

	while(num_of_bytes > 0)
	{
		//Ensure the TxE flag is set
		while(!(Check_Flag(SPI_Handle, SR_TXE_Flag)));

		if(SPI_Handle->SPI_Config.data_frame == Data_8_Bits)
		{
			SPI_Handle->SPIx->DR = *(pTxBuffer);
			num_of_bytes--;
			pTxBuffer++;
		}
	}
	Disable_SPI(SPI_Handle);
}

/*
 * @brief	This funtion utilizes the full-duplex mode to read data from the registers.
 *
 * @note	The process to implement full-duplex communication for the stm32f4 is detailed in the reference
 * 			manual.
 *
 * @param	pRxBuffer: Takes an address of a pointer/array to load data into.
 *
 * @param	num_of_bytes: Specifies the total number of bytes the user wishes to recieve.
 *
 * @param	reg_address: This takes the initial value to write to the MOSI pin which initiates the full-duplex data
 * 			transfer. This would in most cases be the register address to read from.
 */
void SPI_Recieve(SPI_Handle_t *SPI_Handle, uint8_t *pRxBuffer, uint8_t num_of_bytes, uint8_t reg_address)
{
	uint8_t dumm_value = 0x00;

	//Enable SPI peripheral
	SPI_Handle->SPIx->CR1 |= CR1_SPE_Enable;

	//Send the address to read data from
	SPI_Handle->SPIx->DR = reg_address | 0x80;


	while(num_of_bytes > 0)
	{
		//Check TXE flag and send dummy value
		while(!(Check_Flag(SPI_Handle, SR_TXE_Flag)));
		SPI_Handle->SPIx->DR = dumm_value;

		//Ensure the RXNE flag is set
		while(!(Check_Flag(SPI_Handle, SR_RXNE_Flag)));
		*(pRxBuffer) = SPI_Handle->SPIx->DR;
		num_of_bytes--;
		pRxBuffer++;
	}

	Disable_SPI(SPI_Handle);
}




