#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f401_i2c.h"
#include "stm32f401_gpio.h"

#ifndef STM32F401_SPI_H_
#define STM32F401_SPI_H_

typedef struct
{
	uint8_t pin_cs;
	uint8_t pin_clk;
	uint8_t pin_mosi;
	uint8_t pin_miso;
	GPIO_TypeDef *cs_gpio;		//Used only when SSM is enabled
	uint8_t baudrate_ctrl;		//Used to set the clock speed
	uint8_t cpol;				//Used to set clock polarity
	uint8_t cpha;				//Used to set clock phase
	uint8_t data_format;		//Used to send the data either in LSB or MSB form
}SPI_Config_t;

typedef struct
{
	SPI_TypeDef *SPIx;
	SPI_Config_t SPI_Config;
	uint8_t data_frame;			//Used to set data to either 8 or 16 bits
	uint8_t ssm;				//Used to either enable or disable SSM
	uint8_t *pTxBuffer;			//Pointer to a tx buffer that will transmit data to DR
	uint8_t tx_length;			//Used to track of number of bytes to transmit
	uint8_t bus_state;			//Used to keep track of whether SPI is transmitting or receiving data
	uint8_t *pRxBuffer;			//Pointer to the RxBuffer
	uint8_t rx_length;			//Keep track of the number of bytes to receieve
	uint8_t reg_address;
	GPIO_Config_t *Slave;		//Used to hold the instance of the slave - only used in multislave interrupt mode
}SPI_Handle_t;


/*
 * @User Macros
 */

/*Slave select management*/
#define SSM_Disable								(0U)							//Disable slave software management
#define SSM_Enable								(1U)							//Enables slave software management

/*Data framing*/
#define Data_8_Bits								(0U)							//Sets the data frame to 8 bits
#define Data_16_Bits							(1U)							//Sets the data frame to 16 bits
#define MSB_First								(0U << CR1_LSBFIRST_Pos)		//Sends the MSB first
#define	LSB_First								(1U << CR1_LSBFIRST_Pos)		//Sends the LSB first

/*Clock Polarity*/
#define Odd_Polarity							(0U << CR1_CPOL_Pos)			//Clock to 0 when idle
#define Even_Polarity							(1U << CR1_CPOL_Pos)			//Clock to 1 when idle

/*Clock Phase*/
#define Rising_Edge								(0U << CR1_CPHA_Pos)			//first clock transition is first data capture edge
#define Falling_Edge							(1U << CR1_CPHA_Pos)			//Second clock transition is first data capture edge

/*Restart Condition*/
#define Restart									(0U)							//Used if SSM is not set, to decide whether to disable SPI or
#define No_Restart								(1U)							//just use a restart condition

/*Bus state*/
#define SPI_Ready								(0U)
#define SPI_Transmitting						(1U)
#define SPI_Receiving							(2U)

/*Clock Divisor*/
#define DIV2                                    (0U)
#define DIV4  									(1U)
#define DIV8  									(2U)
#define DIV16  									(3U)
#define DIV32  									(4U)
#define DIV64  									(5U)
#define DIV128  								(6U)
#define DIV256  								(7U)

/*
 * @CR1 Flags
 */
#define CR1_BIDIMODE_Enable						(0x1UL << CR1_BIDIMODE_Pos)
#define CR1_BIDIMODE_Pos						(15U)
#define CR1_BIDIOE_Flag							(0x1UL << CR1_BIDIOE_Pos)
#define CR1_BIDIOE_Pos							(14U)
#define CR1_DFF_Enable							(0x1UL << CR1_DFF_Pos)
#define CR1_DFF_Pos								(11U)
#define CR1_RXONLY_Pos							(10U)
#define CR1_RXONLY_Enable						(0x1UL << CR1_RXONLY_Pos)
#define CR1_SSM_Pos								(9U)
#define CR1_SSM_Enable							(0x1UL << CR1_SSM_Pos)
#define CR1_SSI_Pos								(8U)
#define CR1_SSI_Enable							(0x1UL << CR1_SSI_Pos)
#define CR1_LSBFIRST_Pos						(7U)
#define CR1_SPE_Pos								(6U)
#define CR1_SPE_Enable							(0x1UL << CR1_SPE_Pos)
#define CR1_BR_Pos								(3U)
#define CR1_MSTR_Enable							(0x1UL << CR1_MSTR_Pos)
#define CR1_MSTR_Pos							(2U)
#define CR1_CPOL_Pos							(1U)
#define CR1_CPHA_Pos							(0U)

/*
 * @CR2 Flags
 */
#define CR2_TXEIE_Pos							(7U)
#define CR2_TXEIE_Enable						(0x1UL << CR2_TXEIE_Pos)
#define CR2_RXNEIE_Pos							(6U)
#define CR2_RXNEIE_Enable						(0x1UL << CR2_RXNEIE_Pos)
#define CR2_ERRIE_Pos							(5U)
#define CR2_ERRIE_Enable						(0x1UL << CR2_ERRIE_Pos)
#define CR2_SSOE_Pos							(2U)
#define CR2_SSOE_Enable							(0x1UL << CR2_SSOE_Pos)

/*
 * @SR Flags
 */
#define SR_BSY_Pos								(7U)
#define SR_BSY_Flag								(0x1UL << SR_BSY_Pos)
#define SR_OVR_Pos								(6U)
#define SR_OVR_Flag								(0x1UL << SR_OVR_Pos)
#define SR_TXE_Pos								(1U)
#define SR_TXE_Flag								(0x1UL << SR_TXE_Pos)
#define SR_RXNE_Pos								(0U)
#define SR_RXNE_Flag							(0x1UL << SR_RXNE_Pos)

/*
 * @Functions
 */

void SPI_Init(SPI_Handle_t *SPI_Handle);
void SPI_Transmit(SPI_Handle_t *SPI_Handle, uint8_t *pTxBuffer, uint32_t num_of_bytes, uint8_t restart_condition);
void SPI_Receive(SPI_Handle_t *SPI_Handle, uint8_t *pRxBuffer, uint32_t num_of_bytes);
void SPI_TransmitIT(SPI_Handle_t *SPI_Handle, uint8_t *input_buffer, uint8_t num_of_bytes);
void SPI_ReceiveIT(SPI_Handle_t *SPI_Handle, uint8_t *output_buffer, uint8_t num_of_bytes, uint8_t address);
void SPI_MultiSlave_TransmitIT(SPI_Handle_t *SPI_Handle, GPIO_Config_t *Slave_Device, uint8_t *input_buffer, uint8_t num_of_bytes);
void SPI_MultiSlave_RecieveIT(SPI_Handle_t *SPI_Handle, GPIO_Config_t *Slave_Device, uint8_t *output_buffer, uint8_t num_of_bytes, uint8_t address);
void SPI_IRQ_Handler(SPI_Handle_t *SPI_Handle);


#endif /* STM32F401_SPI_H_ */
