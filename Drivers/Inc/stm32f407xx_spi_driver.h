/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Apr 15, 2024
 *      Author: PurplE
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_
#include <stdint.h>
#include "STM32f407xx.h"


/*SPI Pin Configuration
 * */
typedef struct {
	uint8_t SPI_DeviceMode;//Device Mode possible cases : @SPI_DM
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
	uint8_t SPI_SSI;
}SPI_Config_t;
//////////////////////////////////////
/*
 * SPI_APP_state
 */
#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4
/*SPI Handle Structure
 * */
typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxbuffer;
	uint8_t *pRxbuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t  TxState;
	uint8_t  RxState;
}SPI_Handle_t;
/////////////////////
/*@SPI_DeviceMode : Possible Configuration MODES (Master or Slave)
 * */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0
////////////////////////////////
/*
 * @SPI_BUSCONFIG : possible Configuration for Bus (Full-Duplex,Half-Duplex,Simplex)
 * */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3
//////////////////////////////
/*
 * @SPI_SCLK_SPEED :possible Configuration Serial clock
 * */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7
/////////////////////////////////////////
/*
 * @SPI_DFF :Possible Configuration for data format
 * */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1
//////////////////////////////////
/*
 * @SPI_CPOL :Possible Configuration for Clock Polarity
 * */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0
////////////////////////////////////////////
/*
 * @SPI_CPHA :Possible Configuration for Clock Phase
 * */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0
/////////////////////////////////////////////////
/*
 * @SPI_SSM :Possible Configuration for Software slave management
 * */
#define SPI_SSM_DI		0
#define SPI_SSM_EN		1
//////////////////////////////////////////////////
/*
 * @SPI_SSM :Possible Configuration for Software slave management
 * */
#define SPI_SSI_DI		0
#define SPI_SSI_EN		1
//////////////////////////////////////////////////
//FLAG NAEM :
#define SPI_TXE_FLAG		(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1<<SPI_SR_BSY)

/*                              SPI APIs supported by this driver
 * Peripheral clock setup
 * */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*
 * Init and De-init
 * */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint8_t flagName);
void SPI_Init(SPI_Handle_t *SPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 * */
void SPI_SendData(SPI_Handle_t *SPI_handle,uint8_t *pTxBuffer,uint32_t len);
void *SPI_ReceiveData(SPI_Handle_t *SPI_handle,uint8_t *pRxBuffer,uint32_t len);
uint8_t SPI_SendDataIT(SPI_Handle_t *SPI_handle,uint8_t *pTxBuffer,uint32_t len);
uint8_t *SPI_ReceiveDataIT(SPI_Handle_t *SPI_handle,uint8_t *pRxBuffer,uint32_t len);
/*
 * IRQ Configuration and ISR handling
 * */
void SPI_IRQConfing(uint8_t IRQNumber,uint8_t EnOrDi);
void SPI_IRQPriorityConfing(uint8_t IRQNum,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t SPI_handle);

//////////
/*
 *Other functions
 * */
static void spi_txe_interrupt_handle(SPI_Handle_t *SPI_handle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *SPI_handle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *SPI_handle);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);
void SPI_SPEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
