#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f401_uart.h"

#ifndef STM32F401_SPI_H_
#define STM32F401_SPI_H_

typedef struct
{
	uint8_t pin_nss;
	uint8_t pin_mosi;
	uint8_t pin_miso;
	uint8_t pin_sck;
	uint8_t ssm; 				//software slave management pin
	uint8_t ssoe;				//slave select output enable
	uint32_t clock_rate;
	uint8_t data_frame;
	uint8_t lsbfirst;
	uint8_t cpol;
	uint8_t cpha;
	uint8_t spi_bus_direction;
}SPI_Config_t;

typedef struct
{
	SPI_TypeDef *SPIx;
	SPI_Config_t SPI_Config;
	uint16_t *Tx_Buffer;
	uint16_t *Rx_Buffer;
}SPI_Handle_t;

/*
 * @User Macros
 */

/*Slave select management*/
#define SSM_Enable								(1U)		//Enables slave software management
#define	SSOE_Enable								(1U)		//Enables slave select output

/*Data framing*/
#define Data_8_Bits								(0U)		//Sets the data frame to 8 bits
#define Data_16_Bits							(1U)		//Sets the data frame to 16 bits
#define MSB_First								(0U)		//Sends the MSB first
#define	LSB_First								(1U)		//Sends the LSB first

/*SPI bus direction*/
#define SPI_Slave								(0U)		//Sets the MCU to slave device
#define SPI_Master								(1U)		//Sets the MCU to master device

/*Clock Polarity*/
#define Odd_Polarity							(0U)		//Clock to 0 when idle
#define Even_Polarity							(1U)		//Clock to 1 when idle

/*Clock Phase*/
#define Rising_Edge								(0U)		//first clock transition is first data capture edge
#define Falling_Edge							(1U)		//Second clock transition is first data capture edge



/*
 * @CR1 Flags
 */
#define CR1_DFF_Pos								(11U)
#define CR1_SSM_Pos								(9U)
#define CR1_SSM_Enable							(0x1UL << CR1_SSM_Pos)
#define CR1_LSBFIRST_Pos						(7U)
#define CR1_SPE_Pos								(6U)
#define CR1_SPE_Enable							(0x1UL << CR1_SPE_Pos)
#define CR1_BR_Pos								(3U)
#define CR1_MSTR_Pos							(2U)
#define CR1_CPOL_Pos							(1U)
#define CR1_CPHA_Pos							(0U)

/*
 * @CR2 Flags
 */
#define CR2_SSOE_Pos							(2U)
#define CR2_SSOE_Enable							(0x1UL << CR2_SSOE_Pos)

/*
 * @Functions
 */
void SPI_Init(SPI_Handle_t *SPI_Handle);
#endif /* STM32F401_SPI_H_ */
