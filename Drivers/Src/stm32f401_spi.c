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
 * @brief	Helper function used to check flags and disable the SPI peripheral when using slave select output enabled (SSOE)
 */
static void Disable_SPI(SPI_Handle_t *SPI_Handle)
{
	while(!Check_Flag(SPI_Handle, SR_TXE_Flag));
	while(Check_Flag(SPI_Handle, SR_BSY_Flag));
	//Disable SPI
	SPI_Handle->SPIx->CR1 &= ~CR1_SPE_Enable;
}

/*
 * @Brief	SPI initialization function that sets all the configurations such as clock phase, polarity, data frame,
 * 			data format and more.
 */
void SPI_Init(SPI_Handle_t *SPI_Handle)
{
	uint8_t cs_pin, clk_pin, mosi_pin, miso_pin;
	cs_pin = SPI_Handle->SPI_Config.pin_cs;
	clk_pin = SPI_Handle->SPI_Config.pin_clk;
	mosi_pin = SPI_Handle->SPI_Config.pin_mosi;
	miso_pin = SPI_Handle->SPI_Config.pin_miso;

	/*
	 * Set the specified pins based on which SPI peripheral is chosen and whether or not the SSM bit is enabled or
	 * disabled. These functions are called from the "stm32f401_gpio.h" file, and are defined in the stm32f401_gpio.c file.
	 * This also enables clock access to the specified SPI peripheral using functions from the stm32f401_rcc file.
	 */
	if(SPI_Handle->SPIx == SPI1)
	{
		SPI1_Periph_Enable((SPI_Handle->ssm), (SPI_Handle->SPI_Config.cs_gpio), cs_pin, clk_pin, mosi_pin, miso_pin);
		RCC_APB2Cmd(SPI1_Enable, ENABLE);
	}
	else if(SPI_Handle->SPIx == SPI2)
	{
		SPI2_Periph_Enable((SPI_Handle->ssm), (SPI_Handle->SPI_Config.cs_gpio), cs_pin, clk_pin, mosi_pin, miso_pin);
		RCC_APB1Cmd(SPI2_Enable, ENABLE);
	}
	else if(SPI_Handle->SPIx == SPI3)
	{
		SPI3_Periph_Enable((SPI_Handle->ssm), (SPI_Handle->SPI_Config.cs_gpio), cs_pin, clk_pin, mosi_pin, miso_pin);
		RCC_APB1Cmd(SPI3_Enable, ENABLE);
	}
	else
	{
		SPI4_Periph_Enable((SPI_Handle->ssm), (SPI_Handle->SPI_Config.cs_gpio), cs_pin, clk_pin, mosi_pin, miso_pin);
		RCC_APB2Cmd(SPI4_Enable, ENABLE);
	}

	//Set BaudRate control
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->SPI_Config.baudrate_ctrl) << CR1_BR_Pos);

	//Set CPOL and CHPA
	SPI_Handle->SPIx->CR1 |= (SPI_Handle->SPI_Config.cpha);
	SPI_Handle->SPIx->CR1 |= (SPI_Handle->SPI_Config.cpol);

	//Set Data format to either LSB or MSB first
	SPI_Handle->SPIx->CR1 |= (SPI_Handle->SPI_Config.data_format);

	//Set mode to master
	SPI_Handle->SPIx->CR1 |= CR1_MSTR_Enable;

	//Set 8 bit
	SPI_Handle->SPIx->CR1 |= ((SPI_Handle->data_frame) << CR1_DFF_Pos);

	/*
	 * This is used to set the software slave management bit (SSM). If the SSM bit is enabled, this means
	 * the CS will be a seperate GPIO pin specified by the user. If this option is selected SSI is also set high.
	 *
	 * If SSM is not enabled, then SSOE (slave select output enabled) is used which uses the NSS pin as the CS and
	 * it automatically handled by the MCU. When this mode is selected, as soon as the SPI is enabled in master mode, the
	 * CS will go low and will only go high when SPI is disabled. This is why if SSM is enabled, the SPI is enabled also but
	 * if SSOE is enabled it is not enabled in the Init function.
	 */
	if(SPI_Handle->ssm)
	{
		//Set SSM and SSI high
		SPI_Handle->SPIx->CR1 |= ((SPI_Handle->ssm) << CR1_SSM_Pos);
		SPI_Handle->SPIx->CR1 |= CR1_SSI_Enable;
		//Enable SPI periph
		SPI_Handle->SPIx->CR1 |= CR1_SPE_Enable;
	}
	else
	{
		//Set SSOE high
		SPI_Handle->SPIx->CR2 |= CR2_SSOE_Enable;
	}

}

/*
 * @Brief	Funtion to transmit data in full duplex mode using blocking.
 */
void SPI_Transmit(SPI_Handle_t *SPI_Handle, uint8_t *pTxBuffer, uint32_t num_of_bytes, uint8_t restart_condition)
{
	uint32_t temp;
	uint8_t ssm_enabled = SPI_Handle->ssm;

	/* Enable SPI peripheral - This is necessary if the SSM is disabled and SSOE is being used.
	 * As soon as the peripheral is enabled the CS bit it pulled low in SSOE mode.
	 * In SSM mode this is replaced by pulling the specified GPIO pin low.
	 */
	SPI_Handle->SPIx->CR1 |= CR1_SPE_Enable;

	//If data frame is set to 16 bits
	if(SPI_Handle->data_frame == Data_16_Bits)
	{
		while(num_of_bytes > 0)
		{
			//Ensure TXE flag is set and transmit data
			while(!Check_Flag(SPI_Handle, SR_TXE_Flag));
			SPI_Handle->SPIx->DR = *((uint16_t *)pTxBuffer++);
			num_of_bytes--;
		}
	}

	//If data frame is set to 16 bits
	else
	{
		while(num_of_bytes > 0)
		{
			//Ensure TXE flag is set and transmit data
			while(!Check_Flag(SPI_Handle, SR_TXE_Flag));
			SPI_Handle->SPIx->DR = *pTxBuffer++;
			num_of_bytes--;
		}
	}


	while(!(SPI_Handle->SPIx->SR & (SR_TXE_Flag)));
	while(!(SPI_Handle->SPIx->SR & (SR_BSY_Flag)));

	//clear overrun
	temp = SPI_Handle->SPIx->DR;
	temp = SPI_Handle->SPIx->SR;

	/*Used when in SSOE mode. This can prevent the SPI from pulling CS high if restart_condition
	 * is set to restart. This can be used if you want to send an address to read from a slave.
	 */
	if(restart_condition && !ssm_enabled)
	{
		Disable_SPI(SPI_Handle);
	}

}

/*
 * @Brief	Function to receive data in full-duplex mode using blocking.
 */
void SPI_Receive(SPI_Handle_t *SPI_Handle, uint8_t *pRxBuffer, uint32_t num_of_bytes)
{
	uint8_t dummy_byte = 0x00;
	uint8_t ssm_enabled = SPI_Handle->ssm;

	/* Enable SPI peripheral - This is necessary if the SSM is disabled and SSOE is being used.
	 * As soon as the peripheral is enabled the CS bit it pulled low in SSOE mode.
	 * In SSM mode this is replaced by pulling the specified GPIO pin low.
	 */
	SPI_Handle->SPIx->CR1 |= CR1_SPE_Enable;

	//If Data frame is set to 16 bits
	if(SPI_Handle->data_frame == Data_16_Bits)
	{
		while(num_of_bytes)
		{
			//Ensure TXE flag is raised and send dummy_byte
			while(!Check_Flag(SPI_Handle, SR_TXE_Flag)){}
			SPI_Handle->SPIx->DR = (uint16_t)dummy_byte;
			//Ensure RXNE flag is raised and read value from data register
			 while(!Check_Flag(SPI_Handle, SR_RXNE_Flag)){}
			 *((uint16_t *)pRxBuffer++) = SPI_Handle->SPIx->DR;
			 num_of_bytes--;
		}
	}
	//If data frame is set to 8 bits
	else
	{
		while(num_of_bytes)
		{
			//Ensure TXE flag is raised and send dummy_byte
			while(!Check_Flag(SPI_Handle, SR_TXE_Flag)){}
			SPI_Handle->SPIx->DR = dummy_byte;
			//Ensure RXNE flag is raised and read value from data register
			while(!Check_Flag(SPI_Handle, SR_RXNE_Flag)){}
			*pRxBuffer++ = SPI_Handle->SPIx->DR;
			num_of_bytes--;
		}
	}


	//If SSM is disabled then disable the SPI periph
	if(!ssm_enabled)
	{
		Disable_SPI(SPI_Handle);
	}

}

/*
 * @brief	Support function used to enable the NVIC interrupts
 */
static void Enable_NVIC(SPI_Handle_t *SPI_Handle)
{
	if(SPI_Handle->SPIx == SPI1){
		NVIC_EnableIRQ(SPI1_IRQn);
	}
	else if(SPI_Handle->SPIx == SPI2){
		NVIC_EnableIRQ(SPI2_IRQn);
	}
	else if(SPI_Handle->SPIx == SPI3){
		NVIC_EnableIRQ(SPI3_IRQn);
	}
	else{
		NVIC_EnableIRQ(SPI4_IRQn);
	}
}

/*
 * @brief	Support function used to disable interrupt transmission only when SSOE
 * is being used.
 */
static void Disable_SPI_Transmission(SPI_Handle_t *SPI_Handle)
{
	/*Temporary variable to clear the overrun flag - the overrun flag is cleared
	 * by reading the DR followed by the SR.
	 */
	uint32_t clear_overrun;

	while(!Check_Flag(SPI_Handle, SR_TXE_Flag));
	//Disable TXEIE bit in CR2
	SPI_Handle->SPIx->CR2 &= ~CR2_TXEIE_Enable;

	while(Check_Flag(SPI_Handle, SR_BSY_Flag));

	//Clear overrun flag
	clear_overrun = SPI_Handle->SPIx->DR;
	clear_overrun = SPI_Handle->SPIx->SR;

	//Disable SPI periph
	SPI_Handle->SPIx->CR1 &= ~CR1_SPE_Enable;

	//Reset bus state back to SPI_Ready to allow another communication to begin.
	SPI_Handle->bus_state = SPI_Ready;
}

/*
 * @brief	Support function used to disable SPI reception when using interrupts with SSOE.
 */
static void Disable_SPI_Reception(SPI_Handle_t *SPI_Handle)
{
	while(!Check_Flag(SPI_Handle, SR_TXE_Flag));

	//Disable TXEIE and RXNEIE bit
	SPI_Handle->SPIx->CR2 &= ~CR2_TXEIE_Enable;
	SPI_Handle->SPIx->CR2 &= ~CR2_RXNEIE_Enable;

	while(Check_Flag(SPI_Handle, SR_BSY_Flag));

	//Disable SPI periph
	SPI_Handle->SPIx->CR1 &= ~CR1_SPE_Enable;

	//Set the SPI bus state back to SPI_Ready to allow for another seperate SPI transmission
	SPI_Handle->bus_state = SPI_Ready;
}

/*
 * @brief	Support function to transmit data when an interrupt is generated by the TXE flag.
 *
 * @note	This function is called directly from the interrupt service routine.
 */
static void TXE_Interrupt_Handler(SPI_Handle_t *SPI_Handle)
{
	//Check if the bus state is currently in SPI_Transmitting
	if(SPI_Handle->bus_state == SPI_Transmitting)
	{
		//16 bit data mode
		if(SPI_Handle->data_frame == Data_16_Bits)
		{
			if(SPI_Handle->tx_length)
			{
				//Write values from the data buffer into the data register to be transmitted
				SPI_Handle->SPIx->DR = *((uint16_t *)SPI_Handle->pTxBuffer++);
				SPI_Handle->tx_length--;
			}

			if(SPI_Handle->tx_length == 0 && (!SPI_Handle->ssm))
			{
				Disable_SPI_Transmission(SPI_Handle);
			}
		}
		//8 bit data transmission mode
		else
		{
			if(SPI_Handle->tx_length)
			{
				//Write values from the data buffer into the data register to be transmitted
				SPI_Handle->SPIx->DR = *(SPI_Handle->pTxBuffer++);
				SPI_Handle->tx_length--;
			}

			if(SPI_Handle->tx_length == 0)
			{
				/*Once all data bits have been transferred, convert the bus state back to SPI_Ready.
				 * This is also done in the Disable function as a precuation to ensure it is set.
				 */
				SPI_Handle->bus_state = SPI_Ready;

				//If SSOE is being used for slave select
				if(!SPI_Handle->ssm)
				{
					Disable_SPI_Transmission(SPI_Handle);
				}

				/*
				 * If SSM is set high, the following code will pull the chip select pin high again.
				 * It will do so based on the slave device that was passed into the SPI data struct.
				 */
				else
				{
					//Disable TXEIE bit
					SPI_Handle->SPIx->CR2 &= ~CR2_TXEIE_Enable;
					//Pull chip select high
					GPIO_WritePin(SPI_Handle->Slave, GPIO_Write);
				}
			}
		}
	}
	//Used for Receiving data in Full duplex mode - When SPI bus is set to SPI_Receiving
	else
	{
		//Transmit the address of the register to read from
		SPI_Handle->SPIx->DR = SPI_Handle->reg_address;

	}
}

/*
 * @Brief	Support function called from the interrupt service handler when the RXNE flag
 * 			is raised.
 */
static void RXNE_Interrupt_Handler(SPI_Handle_t *SPI_Handle)
{
	//For 16 bit data transmission
	if(SPI_Handle->data_frame == Data_16_Bits)
	{
		//If rx_length is greater than 0
		if(SPI_Handle->rx_length)
		{
			//Read value from Data register inot buffer
			*((uint16_t *)SPI_Handle->pRxBuffer++) = SPI_Handle->SPIx->DR;
			SPI_Handle->rx_length--;
		}
	}

	//For 8 bit data transmission
	else
	{
		//If rx_length is greater than 0
		if(SPI_Handle->rx_length)
		{
			//Read value from Data register inot buffer
			*(SPI_Handle->pRxBuffer++) = SPI_Handle->SPIx->DR;
			SPI_Handle->rx_length--;
		}
	}


	if(SPI_Handle->rx_length == 0)
	{
		//Set the spi bus back to SPI_Ready
		SPI_Handle->bus_state = SPI_Ready;

		//If SSM is disabled
		if(!SPI_Handle->ssm)
		{
			Disable_SPI_Reception(SPI_Handle);
		}

		//If SSM is enabled - pull the specified slave device high to end transmission
		else
		{
			//Disbale the TXEIE and RXNEIE interrupt bits
			SPI_Handle->SPIx->CR2 &= ~CR2_TXEIE_Enable;
			SPI_Handle->SPIx->CR2 &= ~CR2_RXNEIE_Enable;

			//Pull chip select high
			GPIO_WritePin(SPI_Handle->Slave, GPIO_Write);
		}
	}
}

/*
 * @Brief	Function used to initiate SPI transmission using interrupts and with a singular
 * 			slave device.
 */
void SPI_TransmitIT(SPI_Handle_t *SPI_Handle, uint8_t *input_buffer, uint8_t num_of_bytes)
{
	//If bus is not currently transmitting/receiving
	if(SPI_Handle->bus_state == SPI_Ready)
	{
		SPI_Handle->pTxBuffer = input_buffer;
		SPI_Handle->tx_length = num_of_bytes;
		SPI_Handle->bus_state = SPI_Transmitting;

		//Enable NVIC interrupts
		Enable_NVIC(SPI_Handle);

		if(!SPI_Handle->ssm)
		{
			//Enable SPI periph
			SPI_Handle->SPIx->CR1 |= CR1_SPE_Enable;
		}

		//Enable TXEIE bit in CR2
		SPI_Handle->SPIx->CR2 |= CR2_TXEIE_Enable;
	}
}

/*
 * @Brief	Function used to initiate SPI reception using interrupts and with a singular
 * 			slave device.
 */
void SPI_ReceiveIT(SPI_Handle_t *SPI_Handle, uint8_t *output_buffer, uint8_t num_of_bytes, uint8_t address)
{
	//If bus is currently not transmitting/recieving
	if(SPI_Handle->bus_state == SPI_Ready)
	{
		SPI_Handle->pRxBuffer = output_buffer;
		SPI_Handle->rx_length = num_of_bytes;
		SPI_Handle->reg_address = address;
		SPI_Handle->bus_state = SPI_Receiving;

		//Enable NVIC interrupts
		Enable_NVIC(SPI_Handle);

		//Enable TXEIE and RXNEIE bit in CR2
		SPI_Handle->SPIx->CR2 |= CR2_TXEIE_Enable;
		SPI_Handle->SPIx->CR2 |= CR2_RXNEIE_Enable;

		if(!SPI_Handle->ssm)
		{
			//Enable SPI periph
			SPI_Handle->SPIx->CR1 |= CR1_SPE_Enable;
		}


	}
}

/*
 * @Brief	Function used to initiate SPI transmission using interrupts and with the option of single
 * 			or multiple slaves.
 *
 * @note	This requires a GPIO slave device to be passed in, which is created form the GPIO driver included in this file.
 * 			This function will handle pulling the CS low, all that is needed, is to specifcy which slave to interact with.
 */
void SPI_MultiSlave_TransmitIT(SPI_Handle_t *SPI_Handle, GPIO_Config_t *Slave_Device, uint8_t *input_buffer, uint8_t num_of_bytes)
{
	if(SPI_Handle->bus_state == SPI_Ready)
	{
		SPI_Handle->pTxBuffer = input_buffer;
		SPI_Handle->tx_length = num_of_bytes;
		SPI_Handle->bus_state = SPI_Transmitting;
		SPI_Handle->Slave = Slave_Device;

		//Enable NVIC interrupts
		Enable_NVIC(SPI_Handle);

		//Pull Slave Device Low to begin data transmission
		GPIO_WritePin(Slave_Device, GPIO_Reset);

		//Enable TXEIE and RXNEIE bit in CR2
		SPI_Handle->SPIx->CR2 |= CR2_TXEIE_Enable;
	}
}


/*
 * @Brief	Function used to initiate SPI reception using interrupts and with the option of single
 * 			or multiple slaves.
 *
 * @note	This requires a GPIO slave device to be passed in, which is created form the GPIO driver included in this file.
 * 			This function will handle pulling the CS low, all that is needed, is to specifcy which slave to interact with.
 */
void SPI_MultiSlave_RecieveIT(SPI_Handle_t *SPI_Handle, GPIO_Config_t *Slave_Device, uint8_t *output_buffer, uint8_t num_of_bytes, uint8_t address)
{
	if(SPI_Handle->bus_state == SPI_Ready)
	{
		SPI_Handle->pRxBuffer = output_buffer;
		SPI_Handle->rx_length = num_of_bytes;
		SPI_Handle->bus_state = SPI_Receiving;
		SPI_Handle->reg_address = address;
		SPI_Handle->Slave = Slave_Device;

		//Enable NVIC interrupts
		Enable_NVIC(SPI_Handle);

		//Pull Slave Device Low to begin data transmission
		GPIO_WritePin(Slave_Device, GPIO_Reset);

		//Enable TXEIE and RXNEIE bit in CR2
		SPI_Handle->SPIx->CR2 |= CR2_RXNEIE_Enable;
		SPI_Handle->SPIx->CR2 |= CR2_TXEIE_Enable;
	}
}

/*
 * @Brief	SPI interrupt handler routine.
 *
 * @note	This function is called whenever an interrupt occurs generated by the TXE or RXNE flag. It also
 * 			services both the TXE and RXNE interrupts.
 */
void SPI_IRQ_Handler(SPI_Handle_t *SPI_Handle)
{
	uint32_t temp1, temp2;

	temp1 = ((SPI_Handle->SPIx->CR2 & CR2_RXNEIE_Enable) >> 6);
	temp2 = (SPI_Handle->SPIx->SR & SR_RXNE_Flag);
	//If the RXNE flag is raised and the RXNEIE bit is set - call the RXNE interrupt support function
	if(temp1 && temp2)
	{
		RXNE_Interrupt_Handler(SPI_Handle);
	}


	temp1 = ((SPI_Handle->SPIx->CR2 & CR2_TXEIE_Enable) >> 7);
	temp2 = ((SPI_Handle->SPIx->SR & SR_TXE_Flag) >> 1);
	//If the TXE flag is raised and the TXEIE bit is set - call the TXE interrupt support function
	if(temp1 && temp2)
	{
		TXE_Interrupt_Handler(SPI_Handle);
	}


}







