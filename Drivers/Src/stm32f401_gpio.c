#include "stm32f401_rcc.h"
#include "stm32f401_gpio.h"
#include "stm32f4xx.h"


/*This driver was designed to provide the user with easier access to the GPIO ports, and make the
 * process of configuring the ports less arduous. This driver was written specifically for the stm32f401re,
 * and was tested using the stm32f401re nucleo-board. This driver should be functional on any stm32f4xx MCU, however
 * there may be a few differences in some of the functions.
 *
 * The functions within this driver give the user the ability to:
 * 		1)Configure and initialize a specific port or pin based on the desired usage of the GPIO.
 * 		2)Enable or disable (reset) clock access to a GPIO port.
 * 		3)Write a value to the ODR for a pin or write a value to an entire port.
 * 		4)Read values from a pin or an entire port.
 * 		5)Configure the on-board pushbutton and LED (For the stm32f401re nucleo-board)
 * 			using a single function.
 * 		6)Configure an external hardware interrupt.
 *
 * Examples of how to use these functions are present in the Example_Functions folder.
 *
 * Designed By: Kyle Lazera
 */

/*
 * @brief	Configures the GPIO_Config data based on the desired values
 *
 * @note	This function allows the user to initialize multiple structures with different
 * 			values based on how many GPIO pins they wish to use.
 *
 * @param	GPIO_Config_t: Points to the data structure that contains all the configuration
 * 			values for the GPIO pin.
 *
 * @param	Port: Points to the GPIO_TypeDef data structure present in the GPIO_Config_t
 * 			data structure. Defines the specific GPIO port, where x can range from GPIOA - GPIOE,
 * 			or GPIOH.
 *
 * @param	Pin: Accepts a value from the user and assigns it to the pin number in the
 * 			GPIO_Config_t data structure.
 * 			This value should range from 0 - 15.
 * 			It can also hold "AllPins" which is used to initialize the entire port.
 *
 * @para	Mode: Sets the Mode value in the GPIO_Config_t data structure.
 * 			This argument can hold the values:
 * 			0x0		or		GPIO_Input
 * 			0x1		or		GPIO_Output
 * 			0x2		or		GPIO_AF
 * 			0x3		or		GPIO_Analog
 *
 * @param	OType: Sets the Output type value in the GPIO_Config_t data structure.
 * 			This argument can hold the values:
 * 			0x0		or		GPIO_PushPull
 * 			0x1		or		GPIO_OpenDrain
 *
 * @param	OSpeed: Sets the output speed value in the GPIO_Config_t data structure.
 * 			This argument can hold the values:
 * 			0x0		or		GPIO_LowSpeed
 * 			0x1		or		GPIO_MediumSpeed
 * 			0x2		or		GPIO_HighSpeed
 * 			0x3		or		GPIO_VeryHighSpeed
 *
 * @param	PUPD: Sets the Pull up/Pull down value in the GPIO_Config_t data structure
 * 			This argument can hold the values:
 * 			0x0		or		GPIO_PUPD_None
 * 			0x1		or		GPIO_PullUp
 * 			0x2		or		GPIO_PullDown
 */
void GPIO_Config(GPIO_Config_t *GPIO_Config, GPIO_TypeDef *Port, uint8_t Pin, uint8_t Mode, uint8_t OType, uint8_t OSpeed, uint8_t PUPD)
{
	GPIO_Config->GPIO_Pin = Pin;
	GPIO_Config->GPIO_Port = Port;
	GPIO_Config->GPIO_MODE = Mode;
	GPIO_Config->GPIO_OTYPE = OType;
	GPIO_Config->GPIO_OSPEED = OSpeed;
	GPIO_Config->GPIO_PUPD = PUPD;
}

/*
 * @brief	Enables or disables the peripheral clock for the desired GPIO peripheral.
 *
 * @note	Can also be used to reset a GPIO port.
 *
 * @param	GPIOx: Defines the specific GPIO port, where x can range from A - E, or H.
 *
 * @param	state: Specifies whether to enable or disable the peripheral.
 */
void GPIO_PeriphClck(GPIO_TypeDef *GPIOx, FunctionalState state)
{
	if(state == ENABLE)
	{
		if(GPIOx == GPIOA)
		{
			RCC_AHB1Cmd(GPIOA_Enable, ENABLE);
		}

		else if(GPIOx == GPIOB)
		{
			RCC_AHB1Cmd(GPIOB_Enable, ENABLE);
		}

		else if(GPIOx == GPIOC)
		{
			RCC_AHB1Cmd(GPIOC_Enable, ENABLE);
		}

		else if(GPIOx == GPIOD)
		{
			RCC_AHB1Cmd(GPIOD_Enable, ENABLE);
		}

		else if(GPIOx == GPIOE)
		{
			RCC_AHB1Cmd(GPIOE_Enable, ENABLE);
		}

		else if(GPIOx == GPIOH)
		{
			RCC_AHB1Cmd(GPIOH_Enable, ENABLE);
		}

	}

	else
	{
		if(GPIOx == GPIOA)
		{
			RCC_AHB1Cmd(GPIOA_Enable, DISABLE);
		}

		else if(GPIOx == GPIOB)
		{
			RCC_AHB1Cmd(GPIOB_Enable, DISABLE);
		}

		else if(GPIOx == GPIOC)
		{
			RCC_AHB1Cmd(GPIOC_Enable, DISABLE);
		}

		else if(GPIOx == GPIOD)
		{
			RCC_AHB1Cmd(GPIOD_Enable, DISABLE);
		}

		else if(GPIOx == GPIOE)
		{
			RCC_AHB1Cmd(GPIOE_Enable, DISABLE);
		}

		else if(GPIOx == GPIOH)
		{
			RCC_AHB1Cmd(GPIOH_Enable, DISABLE);
		}

	}

}

/*
 * @brief	This function allows the user to set a specific GPIO pin in alternate function mode.
 *
 * @note	See the data-sheet for what each AF value is associated with for each port.
 *
 * @param	GPIO_Config: GPIO_Config: pointer to the GPIO_Config_t data structure that contains the
 * 			configuration parameters for the specific GPIO pin.
 *
 * @param	alt_function: This will take in the alternate function value that the user wishes to use.
 * 			The specific alternate function values and their functions are present in the data sheet.
 * 			The inputs for this function are AF0 - AF15;
 */
void GPIO_AlternateFunctionConfig(GPIO_Config_t *GPIO_Config, AFR_Config_t alt_function)
{
	uint8_t pin = GPIO_Config->GPIO_Pin;
	uint8_t alt_function_array = pin/8;

	if(alt_function_array == 1)
	{
		pin -= 8;
	}

	GPIO_Config->GPIO_Port->AFR[alt_function_array] &= ~(AF15 << (pin * 4));
	GPIO_Config->GPIO_Port->AFR[alt_function_array] |= (alt_function << (pin * 4));
}

/*
 * @brief	Initialize a specific GPIOx pin/port based on the GPIO_Config_t data structure parameters.
 *
 * @note	The RCC_AHB1Cmd() function is defined in the "stm32f401_rcc.c" driver, and enables
 * 			clock access to the specified bit. The definitions to these bits are also defined in
 * 			the "stm32f401_rcc.h" file.
 *
 * @note	To use this function, the GPIO_Config_t data structure needs to have been initialized
 * 			first with the desired parameters. This can be done using the GPIO_Config() function.
 *
 * @note	This function can initialize both a singular pin, or an entire port to the specified
 * 			configuration. To configure the entire port, "AllPins" must be set in the configuration.
 *
 * @note	PA13 and PA14, can not be accessed via the GPIOx control registers. Upon reset, these
 * 			are dedicated pins assigned for debugging purposes which can be used by the
 * 			debugger host. For more details regarding all of the debugging ports, see the Debug Support (DBG)
 * 			section of the reference manual.
 *
 * @note	To allow the user to access all the pins, this function will lock pins PA13 and PA14
 * 			using the lock register to ensure these pins remain unchanged and are used for debugging.
 * 			To unlock these pins, reset the GPIOA peripheral using the GPIO_PeriphClck() function.
 *
 * @param	GPIO_Config: pointer to the GPIO_Config_t data structure that contains the
 * 			configuration parameters for the specific GPIO pin.
 */
void GPIO_Init(GPIO_Config_t *GPIO_Config, AFR_Config_t alt_function)
{
	uint8_t pin = GPIO_Config->GPIO_Pin;
	uint32_t temp_variable;

	GPIO_PeriphClck(GPIO_Config->GPIO_Port, ENABLE);

	if(GPIO_Config->GPIO_Pin == AllPins)
	{
		if(GPIO_Config->GPIO_Port == GPIOA)
		{
			/*Use the lock configuration to lock PA13 and PA14*/
			GPIO_Config->GPIO_Port->LCKR = LCKR_1_Pin13_Pin14;
			GPIO_Config->GPIO_Port->LCKR = LCKR_0_Pin13_Pin14;
			GPIO_Config->GPIO_Port->LCKR = LCKR_1_Pin13_Pin14;
			temp_variable = GPIO_Config->GPIO_Port->LCKR;
		}

		/*Used to set each pin is AllPins is set by user*/
		for(pin = 0; pin < 16; pin++)
		{
			GPIO_Config->GPIO_Port->MODER &= ~(GPIO_Mode_Reset << (pin * 2));
			GPIO_Config->GPIO_Port->MODER |= ((GPIO_Config->GPIO_MODE) << (pin * 2));

			GPIO_Config->GPIO_Port->OTYPER &= ~(GPIO_OType_Reset << pin);
			GPIO_Config->GPIO_Port->OTYPER |= ((GPIO_Config->GPIO_OTYPE) << pin);

			GPIO_Config->GPIO_Port->OSPEEDR &= ~(GPIO_OSpeed_Reset << (pin * 2));
			GPIO_Config->GPIO_Port->OSPEEDR |= ((GPIO_Config->GPIO_OSPEED) << (pin * 2));

			GPIO_Config->GPIO_Port->PUPDR &= ~(GPIO_PUPD_Reset << (pin * 2));
			GPIO_Config->GPIO_Port->PUPDR |= ((GPIO_Config->GPIO_PUPD) << (pin * 2));

		}

	}

	else
	{
		GPIO_Config->GPIO_Port->MODER &= ~(GPIO_Mode_Reset << (pin * 2));
		GPIO_Config->GPIO_Port->MODER |= ((GPIO_Config->GPIO_MODE) << (pin * 2));

		if(GPIO_Config->GPIO_MODE == GPIO_AF)
		{
			GPIO_AlternateFunctionConfig(GPIO_Config, alt_function);
		}

		GPIO_Config->GPIO_Port->OTYPER &= ~(GPIO_OType_Reset << pin);
		GPIO_Config->GPIO_Port->OTYPER |= ((GPIO_Config->GPIO_OTYPE) << pin);

		GPIO_Config->GPIO_Port->OSPEEDR &= ~(GPIO_OSpeed_Reset << (pin * 2));
		GPIO_Config->GPIO_Port->OSPEEDR |= ((GPIO_Config->GPIO_OSPEED) << (pin * 2));

		GPIO_Config->GPIO_Port->PUPDR &= ~(GPIO_PUPD_Reset << (pin * 2));
		GPIO_Config->GPIO_Port->PUPDR |= ((GPIO_Config->GPIO_PUPD) << (pin * 2));
	}

}

/*
 *@brief	Accesses the output data register and allows the user to write data to the specified
 *			GPIO pin.
 *
 *@note		Prior to using this function, the desired GPIO peripheral must be configured and initialized.
 *
 *@param	GPIO_Config: pointer to the GPIO_Config_t data structure that contains the
 * 			configuration parameters for the specific GPIO pin.
 *
 *@param	State: Determines whether the peripheral is turn on, reset, or toggled
 *			This parameter can be:
 *			GPIO_Write		or		0x1
 *			GPIO_Reset		or		0x2
 *			GPIO_Toggle		or		0x3
 */

void GPIO_WritePin(GPIO_Config_t *GPIO_Config, uint8_t State)
{
	uint8_t pin = GPIO_Config->GPIO_Pin;

	if(State == GPIO_Write)
	{
		GPIO_Config->GPIO_Port->ODR |= (0x1UL << pin);
	}

	else if(State == GPIO_Reset)
	{
		GPIO_Config->GPIO_Port->ODR &= ~(0x1UL << pin);
	}

	else if(State == GPIO_Toggle)
	{
		GPIO_Config->GPIO_Port->ODR ^= (0x1UL << pin);
	}

}

/*
 * @brief	Allows the user to write a 16 bit word to the output data register, and access
 * 			all the GPIO pins within the port.
 *
 * @note	For this function to be effective, it is suggested that the entire port be configured
 * 			for output mode.
 *
 * @note	This function is useful when trying to access multiple pins on a port that are all set
 * 			to the same output mode.
 *
 * @note	This function can be used to reset the ODR for a port by writing 0x0000.
 *
 * @param	GPIO_Config: pointer to the GPIO_Config_t data structure that contains the
 * 			configuration parameters for the specific GPIO pin.
 *
 * @param	word: 16-bit data that is to be written to the ODR.
 */
void GPIO_WritePort(GPIO_Config_t *GPIO_Config, uint16_t word)
{
	GPIO_Config->GPIO_Port->ODR = word;
}

/*
 * @brief	Reads a specified pin on the input data register.
 *
 * @param	GPIO_Config: pointer to the GPIO_Config_t data structure that contains the
 * 			configuration parameters for the specific GPIO pin.
 *
 * @retval	Returns either a 1 or 0 from the IDR (Input Data register)
 */
uint8_t GPIO_ReadPin(GPIO_Config_t *GPIO_Config)
{
	uint8_t value;
	uint8_t pin = GPIO_Config->GPIO_Pin;

	value = (uint8_t)(GPIO_IDR_Mask & ((GPIO_Config->GPIO_Port->IDR) >> pin));

	return value;
}

/*
 * @brief	Reads the input data register of a specified port and returns the 16 bit value
 *
 * @param	GPIO_Config: pointer to the GPIO_Config_t data structure that contains the
 * 			configuration parameters for the specific GPIO pin.
 *
 * @retval	Returns a 16 bit value from IDR
 */
uint16_t GPIO_ReadPort(GPIO_Config_t *GPIO_Config)
{
	uint16_t value;

	value = (uint16_t)(GPIO_Config->GPIO_Port->IDR);

	return value;
}


/*
 * @brief	This function specifically activates the on-board LED on the stm32f401RE nucleo-board.
 *
 * @note	This allows the user to access this LED with fewer steps and can be used as a debugging
 * 			tool.
 *
 * @note	This function may not operate correctly is a different version of the nucelo-board is used.
 * 			On the stm32f401RE, the on-board LED is connected to Port A, pin 5.
 *
 *@param	State: Determines whether the peripheral is turn on, reset, or toggled
 *			This parameter can be:
 *			GPIO_Write		or		0x1
 *			GPIO_Reset		or		0x2
 *			GPIO_Toggle		or		0x3
 */
void GPIO_ConfigLEDPA5(uint8_t State)
{
	GPIO_Config_t PortAPin5_LED;
	GPIO_Config(&PortAPin5_LED, GPIOA, Pin5, GPIO_Output, GPIO_PushPull, GPIO_LowSpeed, GPIO_PUPD_None);
	GPIO_Init(&PortAPin5_LED, 0);

	GPIO_WritePin(&PortAPin5_LED, State);
}

/*
 * @brief	This function initializes the on-board pushbutton to receive input.
 *
 * @note	This function was written specifically with the stm32f401RE nucloe-board and may not function
 * 			on a different board.
 *
 * @note	The button is connected to Port C, Pin 13 on the specific nucelo-board.
 *
 * @note	This function allows the user to access the push button with fewer lines of code and can be
 * 			used as an effective form of debugging.
 *
 * @retval	The function returns the value in the IDR for Port C, Pin 13.
 */
uint8_t GPIO_ConfigButtonPC13(void)
{
	GPIO_Config_t PortCPin13_PushButton;
	GPIO_Config(&PortCPin13_PushButton, GPIOC, Pin13, GPIO_Input, 0x0, 0x0, GPIO_PullDown);
	GPIO_Init(&PortCPin13_PushButton, 0);
	return GPIO_ReadPin(&PortCPin13_PushButton);
}

/*
 * @brief	Allows the user to configure an external interrupt using the GPIO pins.
 *
 * @note	This function provides the user with the option of enabling a hardware interrupt
 * 			that is connected to the specified GPIOx pin.
 *
 * @note	To create an interrupt service routine, the user should call the specific IRQHandler
 * 			in their main function, and write the code they would like to be executed in this function.
 * 			See example.
 *
 * @note	There are 16 external interrupt (EXTI) lines on the STM32F401RE, with each line containing a
 * 			specific pin from each port. For example, EXTI0 contains pin 0 from each port (PortA pin0, PortB pin0,
 * 			PortC pin0 etc.). Therefore, the same pin number cannot be set even if the pins belong to different ports.
 * 			For more information see External interrupt/event line mapping in the reference manual.
 *
 * @param	port: This allows the user to select the port the hardware interrupt will be associated with.
 * 			The inputs for this function include:
 * 			EXTI_PortA
 * 			EXTI_PortB
 * 			EXTI_PortC
 * 			EXTI_PortD
 * 			EXTI_PortE
 * 			EXTI_PortH
 *
 * @param	pin: This allows the user to select the specific pin for the hardware interrupt.
 * 			The user can input Pin0 - Pin15, or they can input just the number of the pin.
 * 			Example: Pin13 can be inputed as just 13.
 *
 * @param	edge_trigger: This allows the user to select when the interrupt flag will be raised,
 * 			on the riding edge, falling edge or both.
 * 			The inputs for this function include:
 * 			EXTI_RisingTrigger
 * 			EXTI_FallingTrigger
 * 			EXTI_Rising_FallingTrigger
 *
 */
void GPIO_EXTIConfig(uint8_t port, uint8_t pin, uint8_t edge_trigger)
{
	uint8_t exti_array = pin/4;
	uint8_t temp_pin = pin;

	RCC_APB2Cmd(RCC_APB2ENR_SYSCFGEN, ENABLE);

	switch(exti_array)
	{
	case 1:
		temp_pin -= 4;
		break;

	case 2:
		temp_pin -= 8;
		break;

	case 3:
		temp_pin -= 12;
		break;
	}

	SYSCFG->EXTICR[exti_array] |= (port << (temp_pin * 4));

	EXTI->IMR |= (EXTI_IMR_Set << pin);

	/*Setting the edge trigger as either rising of falling edge*/
	switch(edge_trigger)
	{
	case EXTI_RisingTrigger:
		EXTI->FTSR &= ~(EXTI_Trigger_Set << pin);
		EXTI->RTSR |= (EXTI_Trigger_Set << pin);
		break;

	case EXTI_FallingTrigger:
		EXTI->RTSR &= ~(EXTI_Trigger_Set << pin);
		EXTI->FTSR |= (EXTI_Trigger_Set << pin);
		break;

	case EXTI_Rising_FallingTrigger:
		EXTI->RTSR |= (EXTI_Trigger_Set << pin);
		EXTI->FTSR |= (EXTI_Trigger_Set << pin);
		break;
	}

	/*Enabling a specific EXTI Line*/
	if(pin == 0)
	{
		NVIC_EnableIRQ(EXTI0_IRQn);
	}

	else if(pin == 1)
	{
		NVIC_EnableIRQ(EXTI1_IRQn);
	}

	else if(pin == 2)
	{
		NVIC_EnableIRQ(EXTI2_IRQn);
	}

	else if(pin == 3)
	{
		NVIC_EnableIRQ(EXTI3_IRQn);
	}

	else if(pin == 4)
	{
		NVIC_EnableIRQ(EXTI4_IRQn);
	}

	else if(pin > 4 && pin < 10)
	{
		NVIC_EnableIRQ(EXTI9_5_IRQn);
	}

	else if(pin > 10)
	{
		NVIC_EnableIRQ(EXTI15_10_IRQn);

	}

}

