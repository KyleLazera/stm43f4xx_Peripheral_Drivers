
#ifndef STM32F401_UART_H_
#define STM32F401_UART_H_

#include <stdint.h>
#include "stm32f401_gpio.h"


typedef struct
{
	uint16_t Stop_Bits;
	uint16_t World_Length;
	uint16_t oversampling;
	uint16_t Parity;
	uint32_t BaudRate;
	uint16_t Mode;
	USART_TypeDef *UARTx;
}UART_Config_t;

/*
 * @USART Masks/Configuration Settings
 */

#define UART_CR1_OVER8_Pos       				(15U)
#define UART_CR1_OVER8_Mask      				(0x1UL << USART_CR1_OVER8_Pos)
#define UART_CR1_UE_Pos         			 	(13U)
#define UART_CR1_Enable	         			   	(0x1UL << USART_CR1_UE_Pos)
#define UART_CR1_PS_Pos							(9U)
#define UART_CR1_PS_Mask						(0x1UL << UART_CR1_PS_Pos)
#define UART_CR1_PCE_Pos						(10U)
#define UART_CR1_PCE_Enable						(0x1UL << UART_CR1_PCE_Pos)
#define UART_SR_TXE_Pos							(7U)
#define UART_SR_TXE_Mask						(0x1UL << UART_SR_TXE_Pos)
#define UART_SR_RXNE_Pos						(5U)
#define UART_SR_RXNE_Mask						(0x1UL << UART_SR_RXNE_Pos)
#define UART_SR_ORE_Pos							(3U)
#define UART_SR_ORE_Mask						(0x1UL << UART_SR_ORE_Pos)
#define UART_SR_IDLE_Pos						(4U)
#define UART_SR_IDLE_Mask						(0x1UL << UART_SR_IDLE_Pos)

/*
 * @UART Interrupt Masks
 */

#define UART_CR1_RXNEIE_Pos						(5U)
#define UART_RXNEIE_Enable						(0x1UL << UART_CR1_RXNEIE_Pos)
#define UART_CR1_TXEIE_Pos						(7U)
#define UART_TXEIE_Enable						(0x1UL << UART_CR1_TXEIE_Pos)
#define UART_CR1_TCIE_Pos						(6U)
#define UART_TCIE_Enable						(0x1UL << UART_CR1_TCIE_Pos)

/*
 * @UART_OverSampling
 */

#define UART_OVERSAMPLING_8						((uint16_t)0x8000)
#define UART_OVERSAMPLING_16					((uint16_t)0x0000)

/*
 * @USART_WordLength
 */

#define UART_WORDLEN_8BITS						((uint16_t)0x0000)
#define UART_WORDLEN_9BITS						((uint16_t)0x1000)

/*
 * @USART_StopBits
 */

#define UART_STOPBITS_1							((uint16_t)0x0000)
#define UART_STOPBITS_0_5						((uint16_t)0x1000)
#define UART_STOPBITS_2							((uint16_t)0x2000)
#define UART_STOPBITS_1_5						((uint16_t)0x3000)

/*
 * @USART_TransferDirecton
 */

#define UART_MODE_TX							((uint16_t)0x0008)
#define UART_MODE_RX							((uint16_t)0x0004)
#define UART_MODE_TXRX							((uint16_t)0x000C)

/*
 * @USART_Parity
 */

#define UART_PARITY_DISABLED					((uint16_t)0x0000)
#define UART_PARITY_EVEN						((uint16_t)0x0400)
#define UART_PARITY_ODD							((uint16_t)0x0600)

/*
 * @User- Functions
 */

void UART_Config(UART_Config_t *UART_Config, USART_TypeDef *UARTx, uint16_t mode, uint32_t baudrate);
void UART_Init(UART_Config_t *UART_Config);
void Set_StopBits(UART_Config_t *UART_Config, uint32_t stopbits);
void Set_OverSampling(UART_Config_t *UART_Config, uint32_t oversampling);
void Set_Parity(UART_Config_t *UART_Config, uint32_t parity);
void Set_WordLength(UART_Config_t *UART_Config, uint32_t wordlength);
void WriteByte(UART_Config_t *UART_Config, uint16_t data);
int PrintData(UART_Config_t *UART_Config, char *ptr);
uint8_t ReadByte(UART_Config_t *UART_Config);
void UART_Interrupt_Init(UART_Config_t *UART_Config, uint8_t interrupt_line);

#endif /* STM32F401_UART_H_ */
