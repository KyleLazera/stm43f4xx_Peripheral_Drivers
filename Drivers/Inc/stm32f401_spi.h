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
	uint8_t software_slave_manage;
	uint8_t clock_rate;
	uint8_t data_frame;
	uint8_t lsbfirst;
	uint8_t cpol;
	uint8_t cpha;
}SPI_Config_t;

typedef struct
{
	SPI_TypeDef *SPIx;
	SPI_Config_t SPI_Config;
	uint16_t *Tx_Buffer;
	uint16_t *Rx_Buffer;
}SPI_Handle_t;



#endif /* STM32F401_SPI_H_ */
