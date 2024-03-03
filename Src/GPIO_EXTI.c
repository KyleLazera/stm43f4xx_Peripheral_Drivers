#include <stdint.h>
#include "stm32f401_gpio.h"
#include "stm32f401_rcc.h"
#include "stm32f4xx.h"

/*This example demonstrates how to initialize an external hardware interrupt using the on-board LED
 * and pushbutton on the stm32f401re nucleo-board using the stm32f401_gpio driver.
 *
 * In this program, Port A Pin 5 (on-board LED) is initialized as GPIO output with all other setting at default.
 * Port C Pin 13 (on-board pushbutton) is initialized to input, once again with all other inputs at default. Additionally,
 * The GPIO_EXTIConfig() function is called for Port C Pin 13 and it is set to trigger on the rising edge.
 *
 * By initializing Port C Pin 13, when the pushbutton is pressed, it will activate a flag in the EXTI pending register for pin 13
 * and the interrupt service routine will then run. When initializing an external hardware interrupt, the interrupt service routine
 * for the specific interrupt line must be selected, and this can be used as a function. For this program, since I wanted line 13 to
 * be activated I used "EXTI15_10_IRQHandler." The other interrupt service routines for different lines can be found in the
 * startup_stm32f401retx.s file. In this function, I first cleared the flag in the pending register and then ran the desired code.
 *
 * This program starts off with the on-board LED permanently turned off. Rather than polling the pushbutton state as shown in the GPIO_Pin_Config
 * example, this example uses an external interrupt. When the pushbutton is pressed, the program temporarily stops running the code in the while loop
 * and will run the interrupt service routine, which in this case, turns the on-board LED on temporarily. Once the button is then un-pressed, the
 * LED will return to its original state.
 */

static void EXTI_Callback();
void EXTI15_10_IRQHandler();

GPIO_Config_t PortAPin5;
GPIO_Config_t PortCPin13;

int main()
{
	//Port A, Pin 5 (on-Board LED) - GPIO Output, push-pull, Low Speed and no pull-up/pull-down.
	GPIO_Config(&PortAPin5, GPIOA, Pin5, GPIO_Output, GPIO_PushPull, GPIO_LowSpeed, GPIO_PUPD_None);
	GPIO_Init(&PortAPin5);

	//Port C, Pin 13 (on-board pushbutton) - GPIO Input, push-pull, Medium Speed and Pull-Up activated
	GPIO_Config(&PortCPin13, GPIOC, Pin13, GPIO_Input, GPIO_PushPull, GPIO_LowSpeed, GPIO_PUPD_None);
	GPIO_Init(&PortCPin13);

	//Configure the EXTI
	GPIO_EXTIConfig(EXTI_PortC, Pin13, EXTI_RisingTrigger);

	while(1)
	{
		//Set the default state of the LED to off
		GPIO_WritePin(&PortAPin5, GPIO_Reset);
	}

}

static void EXTI_Callback()
{
	//Turn on the LED/
	GPIO_WritePin(&PortAPin5, GPIO_Write);
	//The for loop is used to add a slight delay so the LED is visible when it is turned on
	for(int i = 0; i < 1000000; i++){}
}

void EXTI15_10_IRQHandler()
{
	//Checking if interrupt flag is raised in pending register
	if((EXTI->PR & (1U << Pin13)) != 0)
	{
		//Clear the PR flag - cleared when a 1 is written to it
		EXTI->PR |= (1U << Pin13);

		EXTI_Callback();

	}

}




