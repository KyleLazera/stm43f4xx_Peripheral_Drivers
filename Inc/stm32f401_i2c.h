#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f401_uart.h"

#ifndef STM32F401_I2C_H_
#define STM32F401_I2C_H_

typedef struct
{
	uint8_t peripheral_clk;
	uint32_t scl_speed;
	uint8_t fm_dutycycle;
	uint8_t pin_scl;
	uint8_t pin_sda;
}I2C_Config_t;

typedef struct
{
	I2C_Config_t I2C_Config;
	I2C_TypeDef *I2Cx;
	uint8_t Tx_Length;
	uint8_t Rx_Length;
	uint8_t *Tx_Buffer;
	uint8_t *Rx_Buffer;
	uint8_t slave_address;
	uint8_t I2C_Bus_Direction;
	uint8_t restart_condition;
}I2C_Handle_t;

typedef enum
{
	Flag_Unset = 0U,
	Flag_Set = 1U
}Flag_Status;

typedef enum
{
	I2C_Recieve = 0U,
	I2C_Transmit = 1U,
	I2C_Ready = 2U
}I2C_Bus;

/*
 * User Macros
 */
#define SM_100KHZ							100000U
#define FM_DUTY_2							(0x0UL << CCR_DUTY_Pos)
#define FM_DUTY_16_9						(0x1UL << CCR_DUTY_Pos)

//Trise register Macros
#define MAX_SM_TRISE_FREQ					1000000U
#define MAX_FM_TRISE_FREQ					3333333U

//Read or Write macros
#define I2C_Write							0U
#define I2C_Read							1U

//Restart Condition
#define I2C_Restart							(1U)
#define I2C_No_Restart						(0U)

/*
 * Control Register 1 (CR1)
 */
#define CR1_SWRST_Pos						(15U)
#define CR1_SWRST							(0x1UL << CR1_SWRST_Pos)
#define CR1_ACK_Pos							(10U)
#define CR1_ACK_Enable						(0x1UL << CR1_ACK_Pos)
#define CR1_STOP_Pos						(9U)
#define CR1_STOP							(0x1UL << CR1_STOP_Pos)
#define CR1_Start_Pos						(8U)
#define CR1_Start							(0x1UL << CR1_Start_Pos)
#define CR1_PE_Pos							(0U)
#define CR1_PE_Enable						(0x1UL << CR1_PE_Pos)

/*
 * Control Register 2 (CR2)
 */
#define CR2_Freq_Pos						(0U)
#define CR2_ITBUFEN_Pos						(10U)
#define CR2_ITBUFEN							(0x1UL << CR2_ITBUFEN_Pos)
#define CR2_ITEVTEN_Pos						(9U)
#define CR2_ITEVTEN							(0x1UL << CR2_ITEVTEN_Pos)
#define CR2_ITERREN_Pos						(8U)
#define CR2_ITERREN							(0x1UL << CR2_ITERREN_Pos)

/*
 * Status Register 1 Flags
 */
#define SR1_SB_Pos							(0U)
#define SR1_SB_Flag							(0x1UL << SR1_SB_Pos)
#define SR1_ADDR_Pos						(1U)
#define SR1_ADDR_Flag						(0x1UL << SR1_ADDR_Pos)
#define SR1_BTF_Pos							(2U)
#define SR1_BTF_Flag						(0x1UL << SR1_BTF_Pos)
#define SR1_RXNE_Pos						(6U)
#define SR1_RXNE_Flag						(0x1UL << SR1_RXNE_Pos)
#define SR1_TXE_Pos							(7U)
#define SR1_TXE_Flag						(0x1UL << SR1_TXE_Pos)

/*
 * Status Register 2 Flags
 */
#define SR2_TRA_Pos							(2U)
#define SR2_TRA_Flag						(0x1UL << SR2_TRA_Pos)
#define SR2_BUSY_Pos						(1U)
#define SR2_BUSY_Flag						(0x1UL << SR2_BUSY_Pos)

/*
 * CCR Register
 */
#define CCR_Speed_Pos						(15U)
#define	CCR_Speed_FM_Mode					(0x1UL << CCR_Speed_Pos)
#define CCR_DUTY_Pos						(14U)
#define CCR_CCR_Pos							(0U)

/*
 * RCC Reset/I2C De-Init
 */
#define I2C1RST_Pos							(21U)
#define I2C1RST								(0x1UL << I2C1RST_Pos)
#define I2C2RST_Pos							(22U)
#define I2C2RST								(0x1UL << I2C2RST_Pos)
#define I2C3RST_Pos							(23U)
#define I2C3RST								(0x1UL << I2C3RST_Pos)



void I2C_Init(I2C_Handle_t *I2C_Handle);
void I2C_DeInit(I2C_Handle_t *I2C_Handle);
void I2C_Config(I2C_Handle_t *I2C_Handle, I2C_TypeDef *I2Cx, uint32_t scl_speed, uint8_t duty_cycle, uint8_t pin_scl, uint8_t pin_sda);
void I2C_MasterTransmit(I2C_Handle_t *I2C_Handle, uint8_t *RxData, uint8_t slave_address, uint8_t number_of_bytes, uint8_t restart_condition);
void I2C_MasterRecieve(I2C_Handle_t *I2C_Handle, uint8_t *RxData, uint8_t slave_address, uint8_t number_of_bytes, uint8_t restart_condition, UART_Config_t *UART_Handle);
void I2C_MasterTransmitIT(I2C_Handle_t *I2C_Handle, uint8_t *TxData, uint8_t slave_address, uint8_t number_of_bytes, uint8_t restart_condition);
void I2C_MasterRecieveIT(I2C_Handle_t *I2C_Handle, uint8_t *TxData, uint8_t slave_address, uint8_t number_of_bytes, uint8_t restart_condition);
void IRQ_Event_Handler(I2C_Handle_t *I2C_Handle, UART_Config_t *UART_Handle);


#endif /* STM32F401_I2C_H_ */
