#include "stm32f401_i2c.h"

/*
 * @brief	A helper function used to activate the correct pins for the I2C peripheral.
 */
static void Enable_I2C_Periph(GPIO_TypeDef *GPIOx, uint8_t pin, AFR_Config_t alt_function)
{
	GPIO_Config_t I2C_Periph;
	GPIO_Config(&I2C_Periph, GPIOx, pin, GPIO_AF, GPIO_OpenDrain, GPIO_LowSpeed, GPIO_PullUp);
	GPIO_Init(&I2C_Periph, alt_function);
}

/*
 * @brief	A helper function that generates the start condition.
 */
static void Generate_Start_Condition(I2C_Handle_t *I2C_Handle)
{
	I2C_Handle->I2Cx->CR1 |= CR1_Start;
}

/*
 * brief	A helper function that keeps track of specific flags in the status register
 */
static Flag_Status Check_Flag(I2C_Handle_t *I2C_Handle, uint32_t flag)
{
	if(I2C_Handle->I2Cx->SR1 & flag)
	{
		return Flag_Set;
	}

	else
	{
		return Flag_Unset;
	}

}

/*
 * brief	A helper function that generates a stop condition
 */
static void Generate_Stop_Condition(I2C_Handle_t *I2C_Handle)
{
	I2C_Handle->I2Cx->CR1 |= CR1_STOP;
}

/*
 * @brief	A helper function used to send the slave address with either a read or write in the LSB
 */
static void Send_Slave_Address(I2C_Handle_t *I2C_Handle)
{
	uint8_t address = I2C_Handle->slave_address;

	//Determine whether read or write mode is active first
	if(I2C_Handle->I2C_Bus_Direction == I2C_Recieve)
	{
		//If the user wishes to read data, append a 1 to the end of the address
		address = ((address << 1) | (I2C_Read << 0U));
		I2C_Handle->I2Cx->DR = address;
	}

	else
	{
		//If the user wishes to write data append a 0 to the end of the address
		address = ((address << 1) | (I2C_Write << 0U));
		I2C_Handle->I2Cx->DR = address;
	}
}

/*
 * @brief	A helper function that clears the Addr flag
 */
static void Clear_Addr_Flag(I2C_Handle_t *I2C_Handle)
{
	uint32_t temporary_variable = 0;

	//Check if there is only 1 byte of data to be recieved and if the I2C is in master recieve mode
	if((I2C_Handle->Rx_Length == 1) && (I2C_Handle->I2C_Bus_Direction == I2C_Recieve))
	{
		//Disable the ACK bit
		I2C_Handle->I2Cx->CR1 &= ~(CR1_ACK_Enable);

		//Clear the addr flag
		temporary_variable = I2C_Handle->I2Cx->SR1;
		temporary_variable = I2C_Handle->I2Cx->SR2;
	}

	else
	{
		temporary_variable = I2C_Handle->I2Cx->SR1;
		temporary_variable = I2C_Handle->I2Cx->SR2;
	}

}

/*
 * @brief	Configures the I2C data structure with user input, avoids use of the dot operator in the main program.
 */
void I2C_Config(I2C_Handle_t *I2C_Handle, I2C_TypeDef *I2Cx, uint32_t scl_speed, uint8_t duty_cycle, uint8_t pin_scl, uint8_t pin_sda)
{
	I2C_Handle->I2Cx = I2Cx;
	I2C_Handle->I2C_Config.scl_speed = scl_speed;
	I2C_Handle->I2C_Config.fm_dutycycle = duty_cycle;
	I2C_Handle->I2C_Config.pin_scl = pin_scl;
	I2C_Handle->I2C_Config.pin_sda = pin_sda;
}

/*
 * @brief	Deinitializes the I2C by resetting the I2Cx bus.
 */
void I2C_DeInit(I2C_Handle_t *I2C_Handle)
{
	if(I2C_Handle->I2Cx == I2C1)
	{
		RCC->APB1RSTR |= I2C1RST;
		RCC->APB1RSTR &= ~I2C1RST;
	}

	else if(I2C_Handle->I2Cx == I2C2)
	{
		RCC->APB1RSTR |= I2C2RST;
		RCC->APB1RSTR &= ~I2C2RST;
	}

	else
	{
		RCC->APB1RSTR |= I2C3RST;
		RCC->APB1RSTR &= ~I2C3RST;
	}
}

/*
 * @brief	Initializes the I2C clock speeds using the CCR and Trise register and turns the I2C1 peripheral on.
 */
void I2C_Init(I2C_Handle_t *I2C_Handle)
{
	RCC_ClockFrequency_t ClockSource;

	uint16_t ccr_value;
	uint8_t trise_value;

	if(I2C_Handle->I2Cx == I2C1)
	{
		//Enable I2C1_SDA Line
		Enable_I2C_Periph(GPIOB, (I2C_Handle->I2C_Config.pin_sda), AF4);
		//Enable I2C1_SCL Line
		Enable_I2C_Periph(GPIOB, (I2C_Handle->I2C_Config.pin_scl), AF4);
		//Enable Clock Access to I2C1
		RCC_APB1Cmd(I2C1_Enable, ENABLE);
	}

	else if(I2C_Handle->I2Cx == I2C2)
	{
		//Enable I2C2_SCL Line for PB10
		Enable_I2C_Periph(GPIOB, 10, AF4);
		//Enable I2C2_SDA Line for PB3
		Enable_I2C_Periph(GPIOB, 3, AF9);
		//Enable Clock Access to I2C2
		RCC_APB1Cmd(I2C2_Enable, ENABLE);
	}

	else
	{
		//Enable I2C3_SCL Line for PA8
		Enable_I2C_Periph(GPIOA, 3, AF4);
		//Enable I2C3_SDA Line for PC9
		Enable_I2C_Periph(GPIOC, 9, AF9);
		//Enable Clock Access to I2C3
		RCC_APB1Cmd(I2C3_Enable, ENABLE);
	}

	I2C_Handle->I2Cx->CR1 |= CR1_SWRST;
	I2C_Handle->I2Cx->CR1 &= ~CR1_SWRST;

	//Determining Peripheral Clock Speed
	RCC_GetClockFreq(&ClockSource);
	I2C_Handle->I2C_Config.peripheral_clk = (ClockSource.PCLCK1)/1000000U & (0x3F);
	I2C_Handle->I2Cx->CR2 |= (I2C_Handle->I2C_Config.peripheral_clk) << CR2_Freq_Pos;

	//CCR Calculations:

	//Check for standard mode
	if(I2C_Handle->I2C_Config.scl_speed <= SM_100KHZ)
	{
		//Clear the fast mode bit ensuring it is set to 0 (set to standard mode)
		I2C_Handle->I2Cx->CCR &= ~CCR_Speed_FM_Mode;
		//In standard mode, the duty cycle is 50%
		ccr_value = (ClockSource.PCLCK1)/(2 * (I2C_Handle->I2C_Config.scl_speed));

		//calculate the Trise value
		trise_value = ((ClockSource.PCLCK1)/(MAX_SM_TRISE_FREQ)) + 1;

	}

	//Check for fast mode
	else
	{
		//Set I2C to fast mode
		I2C_Handle->I2Cx->CCR |= CCR_Speed_FM_Mode;

		if(I2C_Handle->I2C_Config.fm_dutycycle == FM_DUTY_2)
		{
			//Clear the Duty cycle bit so it is set to 0 (which means duty cycle of 2)
			I2C_Handle->I2Cx->CCR &= ~FM_DUTY_16_9;
			ccr_value = ((I2C_Handle->I2C_Config.peripheral_clk)/(3 * I2C_Handle->I2C_Config.scl_speed)) & (0xFFF);
		}

		else
		{
			//Set duty cycle and calculate CCR value
			I2C_Handle->I2Cx->CCR |= FM_DUTY_16_9;
			ccr_value = ((I2C_Handle->I2C_Config.peripheral_clk)/(25 * I2C_Handle->I2C_Config.scl_speed)) & (0xFFF);
		}

		//calculate the Trise value
		trise_value = ((I2C_Handle->I2C_Config.peripheral_clk)/(MAX_FM_TRISE_FREQ)) + 1;

	}

	//Set CCR and Trise register values
	I2C_Handle->I2Cx->CCR |= (ccr_value & 0xFFF) << CCR_CCR_Pos;
	I2C_Handle->I2Cx->TRISE = (trise_value & 0x1F);


	//Enable Peripheral
	I2C_Handle->I2Cx->CR1 |= CR1_PE_Enable;
}


/*
 * brief	A function that sends data from the MCU to the specified slave device - this is a blocking function.
 */
void I2C_MasterTransmit(I2C_Handle_t *I2C_Handle, uint8_t *TxData, uint8_t slave_address, uint8_t number_of_bytes, uint8_t restart_condition)
{
	//Ensure the I2C handle structure is set to transmission
	I2C_Handle->I2C_Bus_Direction = I2C_Transmit;
	I2C_Handle->Tx_Length = number_of_bytes;
	I2C_Handle->slave_address = slave_address;

	//Enable the Acknowledge bit
	//I2C_Handle->I2Cx->CR1 |= CR1_ACK_Enable;

	//Generate Start condition, and wait for the SB bit to be raised in the status register (SR)
	Generate_Start_Condition(I2C_Handle);
	while(!(Check_Flag(I2C_Handle, SR1_SB_Flag)));

	//Send address of slave device with write bit and wait until addr flag is raised
	Send_Slave_Address(I2C_Handle);
	while(!(Check_Flag(I2C_Handle, SR1_ADDR_Flag)));

	//Clear address flag
	Clear_Addr_Flag(I2C_Handle);

	while(number_of_bytes > 0)
	{
		//Ensure the TXE flag is set indicating data transfer is ready to occur
		while(!(Check_Flag(I2C_Handle, SR1_TXE_Flag)));

		//Write to the data register
		I2C_Handle->I2Cx->DR = *TxData;

		//Increment pointer storing the data
		TxData++;

		//Decrement the number of bytes to be transmitted
		number_of_bytes--;
	}

	//Ensure both the BTF and the TXE bit are set indicating both the data register and the shift register are empty
	while(!(Check_Flag(I2C_Handle, SR1_TXE_Flag))){}
	while(!(Check_Flag(I2C_Handle, SR1_BTF_Flag))){}

	//Clear both flags by generating stop condition, unless restart condition is desired
	if(restart_condition == I2C_No_Restart)
	{
		//Generate stop condition
		Generate_Stop_Condition(I2C_Handle);
	}
}

void I2C_MasterRecieve(I2C_Handle_t *I2C_Handle, uint8_t *RxData, uint8_t slave_address, uint8_t number_of_bytes, uint8_t restart_condition, UART_Config_t *UART_Handle)
{
	I2C_Handle->I2C_Bus_Direction = I2C_Recieve;
	I2C_Handle->Rx_Length = number_of_bytes;
	I2C_Handle->slave_address = slave_address;

	//Generate start condition and ensure the SB flag is raised
	Generate_Start_Condition(I2C_Handle);
	while(!(Check_Flag(I2C_Handle, SR1_SB_Flag)));
	PrintData(UART_Handle, "Start Bit Generated\n\r");

	//Send slave address with read as the LSB and ensure the addr flag is raised
	Send_Slave_Address(I2C_Handle);
	while(!(Check_Flag(I2C_Handle, SR1_ADDR_Flag)));
	PrintData(UART_Handle, "Address flag raised.\n\r");

	//Clear the address flag
	Clear_Addr_Flag(I2C_Handle);
	PrintData(UART_Handle, "Address flag cleared.\n\r");

	//Check if only 1 byte of data is to be recieved
	if(number_of_bytes == 1)
	{
		PrintData(UART_Handle, "Rx Length = 1.\n\r");

		//Ensure the RXNE flag is raised
		while(!(Check_Flag(I2C_Handle, SR1_RXNE_Flag)));

		//Check whether the user wants the master to release the SDA and SCL line
		if(restart_condition == I2C_No_Restart)
		{
			//Generate stop condition
			Generate_Stop_Condition(I2C_Handle);
		}

		//Read data from the data regsiter
		*RxData = I2C_Handle->I2Cx->DR;
	}

	for(int i = number_of_bytes; i > 1; i--)
	{
		PrintData(UART_Handle, "Rx Length > 1.\n\r");
		//Ensure the RXNE flag is raised

		while(!(Check_Flag(I2C_Handle, SR1_RXNE_Flag)));

		//Read from the data register
		*RxData = I2C_Handle->I2Cx->DR;

		if(i == 2)
		{
			PrintData(UART_Handle, "Rx length = 2.\n\r");

			//ACK  disabled to generate the NACK
			I2C_Handle->I2Cx->CR1 &= ~CR1_ACK_Enable;

			//Check whether the user wants the master to release the SDA and SCL line
			if(restart_condition == I2C_No_Restart)
			{
				//Generate stop condition
				Generate_Stop_Condition(I2C_Handle);
				PrintData(UART_Handle, "Stop condition.\n\r");
			}
		}

		//Incremenet the data buffer
		RxData++;
	}


	//Enable the ACK bit
	I2C_Handle->I2Cx->CR1 |= CR1_ACK_Enable;
}

/*
 * @brief	Enables the interrupts for the I2Cx peripherals through the NVIC
 */
static void Enable_Interrupt_Handler(I2C_TypeDef *I2Cx)
{	//Check which I2C peripheral is being used and enable its respective I2C error and event interrupt handle
	if(I2Cx == I2C1)
	{
		NVIC_EnableIRQ(I2C1_EV_IRQn);
		NVIC_EnableIRQ(I2C1_ER_IRQn);
	}

	else if(I2Cx == I2C2)
	{
		NVIC_EnableIRQ(I2C2_EV_IRQn);
		NVIC_EnableIRQ(I2C2_ER_IRQn);
	}

	else
	{
		NVIC_EnableIRQ(I2C3_EV_IRQn);
		NVIC_EnableIRQ(I2C3_ER_IRQn);
	}
}

static void MasterTransmit_TxEInterrupt(I2C_Handle_t *I2C_Handle, UART_Config_t *UART_Handle)
{
	PrintData(UART_Handle, "Transmitting data.\n\r");
	//Check if there is data to be transmitted
	if(I2C_Handle->Tx_Length > 0)
	{
		PrintData(UART_Handle, "Tx Length greater than 0.\n\r");
		//Write the data to the DR
		I2C_Handle->I2Cx->DR = *(I2C_Handle->Tx_Buffer);

		//Decrement the length of the data to be transmitted
		I2C_Handle->Tx_Length--;

		//Increment the address of the buffer
		I2C_Handle->Tx_Buffer++;
	}
}

static void MasterRecieve_RXNEInterrupt(I2C_Handle_t *I2C_Handle)
{
	if(I2C_Handle->Rx_Length == 1)
	{
		//Ack bit needs to be disabled before clearing the address flag - this is handled by the clear address flag function
		*(I2C_Handle->Rx_Buffer) = I2C_Handle->I2Cx->DR;

		I2C_Handle->Rx_Length--;
	}

	else if(I2C_Handle->Rx_Length > 1)
	{
		//Check if there are 2 bytes of data to be recieved
		if(I2C_Handle->Rx_Length == 2)
		{
			//Disbale the Ack
			I2C_Handle->I2Cx->CR1 &= ~(CR1_ACK_Enable);
		}

		//Read data from DR
		*(I2C_Handle->Rx_Buffer) = I2C_Handle->I2Cx->DR;

		//Decrement the length of the buffer
		I2C_Handle->Rx_Length--;

		//Increment the address of the buffer
		I2C_Handle->Rx_Buffer++;
	}

	if(I2C_Handle->Rx_Length == 0)
	{
		//Check for restart condition
		if(I2C_Handle->restart_condition == I2C_No_Restart)
		{
			//Generate stop condition
			Generate_Stop_Condition(I2C_Handle);
		}
	}


}

static void I2C_EndDataTransmission(I2C_Handle_t *I2C_Handle)
{
	//Disable the two interrupt enable bits
	I2C_Handle->I2Cx->CR2 &= ~CR2_ITBUFEN;
	I2C_Handle->I2Cx->CR2 &= ~CR2_ITEVTEN;

	//Clear the I2C handle
	I2C_Handle->I2C_Bus_Direction = I2C_Ready;
}

/*
 * @brief
 */
void I2C_MasterTransmitIT(I2C_Handle_t *I2C_Handle, uint8_t *TxData, uint8_t slave_address, uint8_t number_of_bytes, uint8_t restart_condition)
{
	//Enable interrupts for the secified I2C peripheral
	Enable_Interrupt_Handler(I2C_Handle->I2Cx);

	//Ensure the I2C bus is not busy - If it is not set the specs for the I2C data structure
	if(((I2C_Handle->I2C_Bus_Direction)!= I2C_Transmit) && ((I2C_Handle->I2C_Bus_Direction)!= I2C_Recieve))
	{
		I2C_Handle->slave_address = slave_address;
		I2C_Handle->Tx_Length = number_of_bytes;
		I2C_Handle->Tx_Buffer = TxData;
		I2C_Handle->restart_condition = restart_condition;
		I2C_Handle->I2C_Bus_Direction = I2C_Transmit;

		//Enable the Interrupt events
		I2C_Handle->I2Cx->CR2 |= CR2_ITBUFEN;
		I2C_Handle->I2Cx->CR2 |= CR2_ITEVTEN;
		I2C_Handle->I2Cx->CR2 |= CR2_ITERREN;

		//Generate the start condition to start the I2C bus transfer
		Generate_Start_Condition(I2C_Handle);
	}
}

/*
 * @brief
 */
void I2C_MasterRecieveIT(I2C_Handle_t *I2C_Handle, uint8_t *TxData, uint8_t slave_address, uint8_t number_of_bytes, uint8_t restart_condition)
{
	//Enable interrupts for the secified I2C peripheral
	Enable_Interrupt_Handler(I2C_Handle->I2Cx);

	//Ensure the I2C bus is not busy - If it is not set the specs for the I2C data structure
	if(((I2C_Handle->I2C_Bus_Direction)!= I2C_Transmit) && ((I2C_Handle->I2C_Bus_Direction)!= I2C_Recieve))
	{
		I2C_Handle->slave_address = slave_address;
		I2C_Handle->Tx_Length = number_of_bytes;
		I2C_Handle->Tx_Buffer = TxData;
		I2C_Handle->restart_condition = restart_condition;
		I2C_Handle->I2C_Bus_Direction = I2C_Recieve;

		//Enable the Interrupt events
		I2C_Handle->I2Cx->CR2 |= CR2_ITBUFEN;
		I2C_Handle->I2Cx->CR2 |= CR2_ITEVTEN;
		I2C_Handle->I2Cx->CR2 |= CR2_ITERREN;

		//Generate the start condition to start the I2C bus transfer
		Generate_Start_Condition(I2C_Handle);
	}
}

/*
 * @brief
 */
void IRQ_Event_Handler(I2C_Handle_t *I2C_Handle, UART_Config_t *UART_Handle)
{
	//Used to check status of registers
	uint8_t evt_interrupt, buff_interrupt, sr_flag;
	PrintData(UART_Handle, "Inside event handler.\n\r");

	//Used to ensure the interrupt enable bits are set
	evt_interrupt = (I2C_Handle->I2Cx->CR2 & CR2_ITEVTEN) >> 9;
	buff_interrupt = (I2C_Handle->I2Cx->CR2 & CR2_ITBUFEN) >> 10;

	//1)SB flag is raised after start generation
	sr_flag = I2C_Handle->I2Cx->SR1 & SR1_SB_Flag;

	if(evt_interrupt && sr_flag)
	{
		Send_Slave_Address(I2C_Handle);
		PrintData(UART_Handle, "Slave address sent.\n\r");
	}

	//2) ADDR flag is set after the address was sent to the slave device
	sr_flag = (I2C_Handle->I2Cx->SR1 & SR1_ADDR_Flag) >> 1;

	if(evt_interrupt && sr_flag)
	{
		Clear_Addr_Flag(I2C_Handle);
		PrintData(UART_Handle, "Address flag cleared.\n\r");
	}

	//5) BTF flag is set Inidcating byte transfer finished
	//		In transmission: Both the DR and shift register are empty (DR needs to be written)
	//		In reception: Both Dr and shift register are full (DR needs to be read)
	sr_flag = (I2C_Handle->I2Cx->SR1 & SR1_BTF_Flag) >> 2;

	if(evt_interrupt && sr_flag)
	{
		PrintData(UART_Handle, "BTF flag set.\n\r");
		//Check if I2C peripheral is in transmission mode and TxE flag is set - inidcates both BTF and TxE flags are set
		if((I2C_Handle->I2C_Bus_Direction == I2C_Transmit) && ((I2C_Handle->I2Cx->SR1 & SR1_TXE_Flag)>>7))
		{
			if(I2C_Handle->Tx_Length ==0)
			{
				if((I2C_Handle->restart_condition == I2C_No_Restart))
				{
					//Generate stop condition
					Generate_Stop_Condition(I2C_Handle);
					PrintData(UART_Handle, "Stop condition generated.\n\r");
				}

				else
					PrintData(UART_Handle, "Closing Data Transmission.\n\r");
					I2C_EndDataTransmission(I2C_Handle);
			}
		}
	}

	//3) TxE flag is set indicating the data register is empty (in transmission)
	//		- Cleared by software writing to the DR of by hardware after a start/stop condition
	sr_flag = (I2C_Handle->I2Cx->SR1 & SR1_TXE_Flag) >> 7;

	if(evt_interrupt && buff_interrupt && sr_flag)
	{
		MasterTransmit_TxEInterrupt(I2C_Handle, UART_Handle);
		PrintData(UART_Handle, "Txe Flag Cleared.\n\r");
	}

	//4) RxNE flag is set indicating data is in the DR (when reciving data)
	sr_flag = (I2C_Handle->I2Cx->SR1 & SR1_RXNE_Flag) >> 6;

	if(evt_interrupt && buff_interrupt && sr_flag)
	{
		MasterRecieve_RXNEInterrupt(I2C_Handle);
		PrintData(UART_Handle, "RxNE Flag cleared.\n\r");
	}

}
