
#include <stdint.h>
#include "stm32f401_gpio.h"
#include "stm32f401_rcc.h"
#include "stm32f4xx.h"

/*This is an example of how to initialize multiple GPIO pins and how to control their
 * outputs (turn on, off or toggle) and read inputs using the functions present in
 * the stm32f401_gpio.c re-usable driver.
 *
 * Using the stm32f401re nucelo-board, Port A, pin 5 and Port C, pin 13 are connected to
 * the on-board LED and pushbutton respectively. Based on the program I included in the while loop
 * the on-board LED should always be on unless the pushbutton is pressed, in which case it will toggle
 * on and off.
 *
 * To test the other pins that were initialized using the GPIO functions, the debugger can be used and the
 * values written to the registers can be monitored, to ensure they have been set to the correct settings.
 */

uint8_t gpio_read;

int main()
{
	//Port A, Pin 5 (on-Board LED) - GPIO Output, push-pull, Low Speed and no pull-up/pull-down.*
	GPIO_Config_t PortAPin5;
	GPIO_Config(&PortAPin5, GPIOA, Pin5, GPIO_Output, GPIO_PushPull, GPIO_LowSpeed, GPIO_PUPD_None);
	GPIO_Init(&PortAPin5, 0x0);

	// Port C, Pin 13 (on-board pushbutton) - GPIO Input, push-pull, Medium Speed and Pull-Up activated
	GPIO_Config_t PortCPin13;
	GPIO_Config(&PortCPin13, GPIOC, Pin13, GPIO_Input, GPIO_PushPull, GPIO_MediumSpeed, GPIO_PUPD_None);
	GPIO_Init(&PortCPin13, 0x0);

	// Port B, Pin 6 - Alternate Function, push-pull, Low Speed, no pull-up/pull-down, AF02
	GPIO_Config_t PortBPin6;
	GPIO_Config(&PortBPin6, GPIOB, Pin6, GPIO_AF, GPIO_PushPull, GPIO_LowSpeed, GPIO_PUPD_None);
	GPIO_Init(&PortBPin6, AF2);

	//Port D, All pins, analog mode, push-pull, High Speed, No pull-up/pull-down
	GPIO_Config_t PortDAllPins;
	GPIO_Config(&PortDAllPins, GPIOD, AllPins, GPIO_Analog, GPIO_PushPull, GPIO_HighSpeed, GPIO_PUPD_None);
	GPIO_Init(&PortDAllPins);

	while(1)
	{
		gpio_read = GPIO_ReadPin(&PortCPin13);
		if(gpio_read == 1)
		{
			GPIO_WritePin(&PortAPin5, GPIO_Write);
		}

		else
		{
			GPIO_WritePin(&PortAPin5, GPIO_Toggle);
			//Used to add a temporary delay so the toggling is visible
			for(int i = 0; i < 100000; i++);
		}
	}
}





