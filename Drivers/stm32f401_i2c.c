#include "stm32f401_i2c.h"

/*
 * This is a simple I2C driver with the main purpose of stream-lining initilization and transmission/reception from the stm32f4xx MCU.
 * The program allows the user to configure the I2C settings such as the SCL speed, the duty cycle and which pins to use. Additionally, the
 * user can initialize the I2C with these settings and then transmit and recieve data to and from slave devices. This program also gives the
 * user the option of blocking/not-blocking the CPU during I2C communication, with the inclusion of an interrupt initialization and event handler
 * function.
 *
 * The functions within this driver give the user the ability to:
 * 		1) Configure and initialize the specified I2C peripheral with the desired settings (clock speed, duty cycle etc.).
 * 		2) Transmit single or multiple bytes of data to from the MCU to the slave device with or without blocking CPU.
 * 		3) Recieve single or multiple bytes of data from the slave device with or without blocking the CPU.
 *
 *  Designed by: Kyle Lazera
 */


/*Helper Functions*/
static void Enable_I2C_Periph(GPIO_TypeDef *GPIOx, uint8_t pin, AFR_Config_t alt_function);
static void Generate_Start_Condition(I2C_Handle_t *I2C_Handle);
static Flag_Status Check_Flag(I2C_Handle_t *I2C_Handle, uint32_t flag);
static void Generate_Stop_Condition(I2C_Handle_t *I2C_Handle);
static void Send_Slave_Address(I2C_Handle_t *I2C_Handle);
static void Clear_Addr_Flag(I2C_Handle_t *I2C_Handle);


/*
 * @brief	A helper function used to set GPIO pins to correct settings to use I2C peripheral.
 *
 * @note	Sets the specified pins to alternate function, open drain, low speed and pull-up resistor.
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
 * brief	A helper function that keeps track of specific flags in the status register.
 *
 * @retval	Returns status of the flag which is defined in an enumeration.
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
 * @brief	A helper function used to send the slave address with either a read or write in the LSB.
 */
static void Send_Slave_Address(I2C_Handle_t *I2C_Handle)
{
	uint8_t slave_address = I2C_Handle->slave_address;

	//Determine whether read or write mode is active first
	if(I2C_Handle->I2C_Bus_Direction == I2C_Recieve)
	{
		//If the user wishes to read data, append a 1 to the end of the address
		slave_address = ((slave_address << 1) | (I2C_Read << 0U));
		I2C_Handle->I2Cx->DR = slave_address;
	}

	else
	{
		//If the user wishes to write data append a 0 to the end of the address
		slave_address = ((slave_address << 1) | (I2C_Write << 0U));
		I2C_Handle->I2Cx->DR = slave_address;
	}

}

/*
 * @brief	A helper function that clears the Addr flag.
 *
 * @note	If the I2C peripheral is in receiver mode and is set to only recieve 1 byte, the ACK
 * 			bit will be disabled before clearning addr flag.
 */
static void Clear_Addr_Flag(I2C_Handle_t *I2C_Handle)
{
	uint32_t temporary_variable = 0;

	if((I2C_Handle->I2C_Bus_Direction == I2C_Recieve) && (I2C_Handle->Rx_Length == 1))
	{
		//Disable the Ack
		I2C_Handle->I2Cx->CR1 &= ~(CR1_ACK_Enable);

		//Clear addr flag
		temporary_variable = I2C_Handle->I2Cx->SR1;
		temporary_variable = I2C_Handle->I2Cx->SR2;

	}

	else
	{
		//Clear the addr flag
		temporary_variable = I2C_Handle->I2Cx->SR1;
		temporary_variable = I2C_Handle->I2Cx->SR2;
	}


}

/*
 * @brief	Configures the I2C data structure with user input.
 *
 * @note	If I2C2 or I2C3 is specified, pin_scl and pin_sda parameters are not used.
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
 * @brief	Deinitializes the I2C by turning off the I2C access at the APB1ENR.
 */
void I2C_DeInit(I2C_Handle_t *I2C_Handle)
{
	if(I2C_Handle->I2Cx == I2C1)
	{
		RCC->APB1ENR &= ~I2C1EN;
	}

	else if(I2C_Handle->I2Cx == I2C2)
	{
		RCC->APB1ENR &= ~I2C2EN;
	}

	else
	{
		RCC->APB1ENR &= ~I2C3EN;
	}
}

/*
 * @brief	Initializes the I2C clock speeds using the CCR and Trise register and turns the I2C1 peripheral on.
 *
 * @note	The equations used to determine the CCR register value differs based on whether standard or fast mode
 * 			is selected by the user. Additionally, If fast mode is selected the equations will differ based on
 * 			the duty cycle selected.
 *
 * @note	To avoid using float or double data types, the equations were rearranged to use the frequency values
 * 			as opposed to the period of the peripheral clock. This is done by using the relationship between period
 * 			and frequency (T = 1/F).
 *
 * @note	For standard Input mode (SM) the peripheral input clock must be at least 2MHz and for Fast mode (FM)
 * 			the peripheral input clock must be at least 4MHz.
 *
 * @note	The equation to determine the Trise register value is (T(scl)/T(pclck)) + 1, where the
 * 			T(scl) is the maximum SCL rise time and T(pclck) is the period of the peripheral bus.
 * 			Based on the data sheet, the T(scl) for SM mode is 1000ns and for FM mode is 300ns.
 *
 * @note	Using the property that T = 1/F, where T is the period and F is the frequency, the equation
 * 			to calculate the Trise value can be rearranged to get (F(pclck)/F(scl)) + 1, where F(pclck) is the
 * 			frequency of the peripheral clock and F(scl) is the maximum frequency for the rise time.
 *
 * @note	For standard mode F(scl) is 1000000 and for fast mode F(scl) is 3333333. These values are defined in
 * 			the header file as MAX_SM_TRISE_FREQ and MAX_FM_TRISE_FREQ.
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

	//I2C_Handle->I2Cx->CR1 |= CR1_SWRST;
	//I2C_Handle->I2Cx->CR1 &= ~CR1_SWRST;

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
 *
 * @param	I2C_Handle: A pointer to the data structure that holds the initialization information for the I2C
 * 			peripheral.
 *
 * @param	RxData: This is the data that will be sent from the master to the slave.
 *
 * @param	slave_address:  This will be the address of the peripheral to communicate with.
 *
 * @param	number_of_bytes: This will be the length of the data transfer.
 *
 * @param 	restart_condition: This will be used to impact whether or not the master releases the SDA or SCL line
 * 			by sending a stop condition.
 */
void I2C_MasterTransmit(I2C_Handle_t *I2C_Handle, uint8_t *TxData, uint8_t slave_address, uint8_t number_of_bytes, uint8_t restart_condition)
{
	I2C_Handle->I2C_Bus_Direction = I2C_Transmit;
	I2C_Handle->slave_address = slave_address;

	//Enable the ACK bit
	I2C_Handle->I2Cx->CR1 |= CR1_ACK_Enable;

	//Generate start condition
	Generate_Start_Condition(I2C_Handle);
	//Ensure the SB flag is set indictaing start condition was generated
	while(!(Check_Flag(I2C_Handle, SR1_SB_Flag)));

	//Send slave address
	Send_Slave_Address(I2C_Handle);
	//Ensure the addr flag has been set indicating the end of address transmission
	while(!(Check_Flag(I2C_Handle, SR1_ADDR_Flag)));

	//Clear the address flag
	Clear_Addr_Flag(I2C_Handle);

	while(number_of_bytes > 0)
	{
		while(!(Check_Flag(I2C_Handle, SR1_TXE_Flag)));

		I2C_Handle->I2Cx->DR = *TxData;

		number_of_bytes--;

		TxData++;
	}

	while(!(Check_Flag(I2C_Handle, SR1_TXE_Flag)));
	while(!(Check_Flag(I2C_Handle, SR1_BTF_Flag)));

	if(restart_condition == I2C_No_Restart)
	{
		Generate_Stop_Condition(I2C_Handle);
		I2C_Handle->I2C_Bus_Direction = I2C_Ready;
	}

}

/*
 * @brief A function that recieves data from the slave - this is a blocking function.
 *
 * @param	I2C_Handle: A pointer to the data structure that holds the initialization information for the I2C
 * 			peripheral.
 *
 * @param	RxData: This is the buffer that will store the data read from the MCU.
 *
 * @param	slave_address: This will be the address of the peripheral to communicate with.
 *
 * @param	number_of_bytes: This will be the length of the data transfer.
 *
 * @param	restart_condition: This will be used to impact whether or not the master releases the SDA or SCL line
 * 			by sending a stop condition.
 */
void I2C_MasterRecieve(I2C_Handle_t *I2C_Handle, uint8_t *RxData, uint8_t slave_address, uint8_t number_of_bytes, uint8_t restart_condition)
{
	I2C_Handle->I2C_Bus_Direction = I2C_Recieve;
	I2C_Handle->slave_address = slave_address;
	I2C_Handle->Rx_Length = number_of_bytes;

	//Generate start condition and ensure the SB flag is raised
	Generate_Start_Condition(I2C_Handle);
	while(!(Check_Flag(I2C_Handle, SR1_SB_Flag)));

	//Send slave address with read as the LSB and ensure the addr flag is raised
	Send_Slave_Address(I2C_Handle);
	while(!(Check_Flag(I2C_Handle, SR1_ADDR_Flag)));

	//Clear addr flag
	Clear_Addr_Flag(I2C_Handle);

	//Check if number of bytes to be transmitted is 1
	if(number_of_bytes == 1)
	{

		//wait until RXNE flag is raised
		while(!(Check_Flag(I2C_Handle, SR1_RXNE_Flag)));

		if(restart_condition == I2C_No_Restart)
		{
			//Generate stop condition
			Generate_Stop_Condition(I2C_Handle);
			I2C_Handle->I2C_Bus_Direction = I2C_Ready;
		}

		//Read data
		*RxData = I2C_Handle->I2Cx->DR;
	}

	//Check if number of bytes to be read is greater than 1
	if(number_of_bytes > 1)
	{

		while(number_of_bytes > 0)
		{

			while(!(Check_Flag(I2C_Handle, SR1_RXNE_Flag)));

			*RxData = I2C_Handle->I2Cx->DR;

			if(number_of_bytes == 2)
			{
				I2C_Handle->I2Cx->CR1 &= ~(CR1_ACK_Enable);

				if(restart_condition == I2C_No_Restart)
				{
					//Generate stop condition
					Generate_Stop_Condition(I2C_Handle);
					I2C_Handle->I2C_Bus_Direction = I2C_Ready;
				}
			}

			RxData++;

			number_of_bytes--;

		}
	}

	I2C_Handle->I2Cx->CR1 |= CR1_ACK_Enable;
}


/*
 * I2C Interrupt Hepler Functions
 */
static void Enable_Interrupt_Handler(I2C_TypeDef *I2Cx);
static void I2C_EndDataTransmission(I2C_Handle_t *I2C_Handle);
static void MasterTransmit_TxEInterrupt(I2C_Handle_t *I2C_Handle);
static void MasterTransmit_TxEInterrupt(I2C_Handle_t *I2C_Handle);

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

/*
 * @brief	Used to close data transmission when transmitting or recieving data with interrupts.
 *
 * @note	Disables the interrupt bits and rests the I2C handle ready flag.
 */
static void I2C_EndDataTransmission(I2C_Handle_t *I2C_Handle)
{
	//Disable the interrupt bits
	I2C_Handle->I2Cx->CR2 &= ~CR2_ITBUFEN;
	I2C_Handle->I2Cx->CR2 &= ~CR2_ITEVTEN;
	I2C_Handle->I2Cx->CR2 &= ~CR2_ITERREN;

	//Clear the I2C handle
	I2C_Handle->I2C_Bus_Direction = I2C_Ready;

	//Enable the ACK
	I2C_Handle->I2Cx->CR1 |= CR1_ACK_Enable;
}

/*
 * @brief	Used to clear the TXE flag and transmit data from the master to the slave.
 */
static void MasterTransmit_TxEInterrupt(I2C_Handle_t *I2C_Handle)
{
	//Check if there is data to be transmitted
	if(I2C_Handle->Tx_Length > 0)
	{
		//Write the data to the DR
		I2C_Handle->I2Cx->DR = *(I2C_Handle->Tx_Buffer);

		//Decrement the length of the data to be transmitted
		I2C_Handle->Tx_Length--;

		//Increment the address of the buffer
		I2C_Handle->Tx_Buffer++;
	}
}

/*
 * @brief	Helper function used to clear the RXNE flag by reading data from the slave.
 */
static void MasterRecieve_RXNEInterrupt(I2C_Handle_t *I2C_Handle)
{

	if(I2C_Handle->Rx_Size == 1)
	{

		Generate_Stop_Condition(I2C_Handle);

		//Ack bit needs to be disabled before clearing the address flag - this is handled by the clear address flag function
		*(I2C_Handle->Rx_Buffer) = I2C_Handle->I2Cx->DR;

		I2C_EndDataTransmission(I2C_Handle);
	}

	else if(I2C_Handle->Rx_Size > 1)
	{

		//Check if there are 2 bytes of data to be recieved
		if(I2C_Handle->Rx_Length == 2)
		{

			//Disbale the Ack
			I2C_Handle->I2Cx->CR1 &= ~(CR1_ACK_Enable);

		}

		//Decrement the length of the buffer
		I2C_Handle->Rx_Length--;

		//Increment the address of the buffer
		I2C_Handle->Rx_Buffer++;

		//Read data from DR
		*(I2C_Handle->Rx_Buffer) = I2C_Handle->I2Cx->DR;

		//If it is the last byte of data to be recieved
		if(I2C_Handle->Rx_Length == 0)
		{
			//Generate stop condition
			Generate_Stop_Condition(I2C_Handle);

			I2C_EndDataTransmission(I2C_Handle);
		}
	}

}

/*
 * @brief Function to intialize data transmission using interrupts - This is a non-blocking function.
 */
void I2C_MasterTransmitIT(I2C_Handle_t *I2C_Handle, uint8_t *TxData, uint8_t slave_address, uint8_t number_of_bytes)
{
	//Enable interrupts for the secified I2C peripheral
	Enable_Interrupt_Handler(I2C_Handle->I2Cx);

	//Ensure the I2C bus is not busy - If it is not set the specs for the I2C data structure
	if(((I2C_Handle->I2C_Bus_Direction)!= I2C_Transmit) && ((I2C_Handle->I2C_Bus_Direction)!= I2C_Recieve))
	{
		I2C_Handle->slave_address = slave_address;
		I2C_Handle->Tx_Length = number_of_bytes;
		I2C_Handle->Tx_Buffer = TxData;
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
 * @brief Function to intialize data reception using interrupts - This is a non-blocking function.
 */
void I2C_MasterRecieveIT(I2C_Handle_t *I2C_Handle, uint8_t *RxData, uint8_t slave_address, uint8_t number_of_bytes)
{
	//Enable interrupts for the secified I2C peripheral
	Enable_Interrupt_Handler(I2C_Handle->I2Cx);

	//Ensure the I2C bus is not busy - If it is not set the specs for the I2C data structure
	if(((I2C_Handle->I2C_Bus_Direction)!= I2C_Transmit) && ((I2C_Handle->I2C_Bus_Direction)!= I2C_Recieve))
	{
		I2C_Handle->slave_address = slave_address;
		I2C_Handle->Rx_Length = number_of_bytes;
		I2C_Handle->Rx_Size = number_of_bytes;
		I2C_Handle->Rx_Buffer = RxData;
		I2C_Handle->I2C_Bus_Direction = I2C_Recieve;

		//Enable the Interrupt events
		I2C_Handle->I2Cx->CR2 |= CR2_ITBUFEN;
		I2C_Handle->I2Cx->CR2 |= CR2_ITEVTEN;
		I2C_Handle->I2Cx->CR2 |= CR2_ITERREN;

		//Enable the ACK bit
		I2C_Handle->I2Cx->CR1 |= CR1_ACK_Enable;

		//Generate the start condition to start the I2C bus transfer
		Generate_Start_Condition(I2C_Handle);
	}
}

/*
 * @brief 	Event handler that handles interrupts during I2C transmission or reception.
 *
 * @note	This handler will clear all flags during transmission.
 */
void IRQ_Event_Handler(I2C_Handle_t *I2C_Handle)
{
	//Used to check status of registers
	uint8_t evt_interrupt, buff_interrupt, sr_flag;

	//Used to ensure the interrupt enable bits are set
	evt_interrupt = (I2C_Handle->I2Cx->CR2 & CR2_ITEVTEN) >> 9;
	buff_interrupt = (I2C_Handle->I2Cx->CR2 & CR2_ITBUFEN) >> 10;

	//1)SB flag is raised after start generation
	sr_flag = I2C_Handle->I2Cx->SR1 & SR1_SB_Flag;

	if(evt_interrupt && sr_flag)
	{
		Send_Slave_Address(I2C_Handle);
	}

	//2) ADDR flag is set after the address was sent to the slave device
	sr_flag = (I2C_Handle->I2Cx->SR1 & SR1_ADDR_Flag) >> 1;

	if(evt_interrupt && sr_flag)
	{
		Clear_Addr_Flag(I2C_Handle);
	}

	//3) BTF flag is set Inidcating byte transfer finished
	//		In transmission: Both the DR and shift register are empty (DR needs to be written)
	//		In reception: Both Dr and shift register are full (DR needs to be read)
	sr_flag = (I2C_Handle->I2Cx->SR1 & SR1_BTF_Flag) >> 2;

	if(evt_interrupt && sr_flag)
	{
		//Check if I2C peripheral is in transmission mode and TxE flag is set - inidcates both BTF and TxE flags are set
		if((I2C_Handle->I2C_Bus_Direction == I2C_Transmit) && ((I2C_Handle->I2Cx->SR1 & SR1_TXE_Flag)>>7))
		{
			if(I2C_Handle->Tx_Length ==0)
			{
				//Generate stop condition
				Generate_Stop_Condition(I2C_Handle);


				I2C_EndDataTransmission(I2C_Handle);
			}
		}
	}

	//4) TxE flag is set indicating the data register is empty (in transmission)
	//		- Cleared by software writing to the DR of by hardware after a start/stop condition
	sr_flag = (I2C_Handle->I2Cx->SR1 & SR1_TXE_Flag) >> 7;

	if(evt_interrupt && buff_interrupt && sr_flag)
	{
		MasterTransmit_TxEInterrupt(I2C_Handle);
	}

	//5) RxNE flag is set indicating data is in the DR (when reciving data)
	sr_flag = (I2C_Handle->I2Cx->SR1 & SR1_RXNE_Flag) >> 6;

	if(evt_interrupt && buff_interrupt && sr_flag)
	{
		MasterRecieve_RXNEInterrupt(I2C_Handle);
	}
}

/*
 * @brief	A function that handles all interrupts generated by error flags.
 *
 * @note	To use this function a UART handle must be created and some form of serial capture
 * 			software to recieve the messages.
 */
void IRQ_Error_Handler(I2C_Handle_t *I2C_Handle, UART_Config_t *UART_Handle)
{
	uint8_t err_interrupt, sr_flag;
	err_interrupt = (I2C_Handle->I2Cx->CR2 & CR2_ITERREN) >> 8;

	//1) Bus error check - Indicates a misplaced start or stop condition
	sr_flag = (I2C_Handle->I2Cx->SR1 & SR1_BERR_Flag) >> 8;

	if(sr_flag && err_interrupt)
	{
		PrintData(UART_Handle, "Bus error: Misplaced start or stop condition detected.\n\r");
		I2C_EndDataTransmission(I2C_Handle);
	}

	// 2) Arbitration lost to another master
	sr_flag =(I2C_Handle->I2Cx->CR2 & SR1_ARLO_Flag) >> 9;

	if(sr_flag && err_interrupt)
	{
		PrintData(UART_Handle, "Error: Arbitration lost to another master.\n\r");
		I2C_EndDataTransmission(I2C_Handle);
	}

	// 3)Acknoweledge Failure
	sr_flag = (I2C_Handle->I2Cx->CR2 & SR1_AF_Flag) >> 10;

	if(sr_flag && err_interrupt)
	{
		PrintData(UART_Handle, "Error: Acknowledge Failure.\n\r");
		I2C_EndDataTransmission(I2C_Handle);
	}

	//4)Overrun or Underrun error
	sr_flag = (I2C_Handle->I2Cx->CR2 & SR1_OVR_Flag) >> 11;

	if(sr_flag && err_interrupt)
	{
		PrintData(UART_Handle, "Error: Overrun or underrun error.\n\r");
		I2C_EndDataTransmission(I2C_Handle);
	}

	//5) PEC error in reception
	sr_flag = (I2C_Handle->I2Cx->CR2 & SR1_PECERR_Flag) >> 12;

	if(sr_flag && err_interrupt)
	{
		PrintData(UART_Handle, "Error: PEC error in reception. Reciever returns NACK after PEC.\n\r");
		I2C_EndDataTransmission(I2C_Handle);
	}

	//6) Timeout error
	sr_flag = (I2C_Handle->I2Cx->CR2 & SR1_TIMEOUT_Flag) >> 14;

	if(sr_flag && err_interrupt)
	{
		PrintData(UART_Handle, "Error: Timeout error. See reference manual for possible causes..\n\r");
		I2C_EndDataTransmission(I2C_Handle);
	}

	//7) SMBAlert error
	sr_flag = (I2C_Handle->I2Cx->CR2 & SR1_SMBALERT_Flag) >> 15;

	if(sr_flag && err_interrupt)
	{
		PrintData(UART_Handle, "Error: SMB Bus error. See reference manual for possible causes.\n\r");
		I2C_EndDataTransmission(I2C_Handle);
	}

}
