/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Apr 15, 2024
 *      Author: PurplE
 */
#include <stdint.h>
#include "STM32f407xx.h"
#include "stm32f407xx_spi_driver.h"
/*                              SPI APIs supported by this driver
 * Peripheral clock setup
 * */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi){
		if(pSPIx==SPI1){
					SPI1_PCLK_EN();
		}
		if(pSPIx==SPI2){
					SPI2_PCLK_EN();
				}
		if(pSPIx==SPI3){
					SPI3_PCLK_EN();
				}
		if(pSPIx==SPI4){
					SPI4_PCLK_EN();
				}
		if(pSPIx==SPI5){
					SPI5_PCLK_EN();
				}
		if(pSPIx==SPI6){
					SPI6_PCLK_EN();
				}
	}else{
		if(pSPIx==SPI1){
					SPI1_PCLK_DI();
				}
				if(pSPIx==SPI2){
							SPI2_PCLK_DI();
						}
				if(pSPIx==SPI3){
							SPI3_PCLK_DI();
						}
				if(pSPIx==SPI4){
							SPI4_PCLK_DI();
						}
				if(pSPIx==SPI5){
							SPI5_PCLK_DI();
						}
				if(pSPIx==SPI6){
							SPI6_PCLK_DI();
						}
	}
}

/*
 * Init and De-init
 * */
void SPI_Init(SPI_Handle_t *SPI_Handle){
	SPI_PeriClockControl(SPI_Handle->pSPIx,ENABLE);
	uint32_t tempReg=0;
	tempReg |= SPI_Handle->SPIConfig.SPI_DeviceMode <<SPI_CR1_MSTR;
	if(SPI_Handle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD){
		//bidi mode should be cleared
		tempReg &=~(1<<SPI_CR1_BIDIMODE);
	}else if(SPI_Handle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD){
		//bidi mode should be Set
		tempReg |=(1<<SPI_CR1_BIDIMODE);
	}else if(SPI_Handle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//bidi mode should be cleared
		tempReg &=~(1<<SPI_CR1_BIDIMODE);
		//Rx only enable
		tempReg |=(1<<SPI_CR1_BIDIOE);
	}
	tempReg |= (SPI_Handle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);
	tempReg |= (SPI_Handle->SPIConfig.SPI_DFF << SPI_CR1_DFF);
	tempReg |= (SPI_Handle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	tempReg |= (SPI_Handle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
	tempReg |= (SPI_Handle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
	tempReg |= (SPI_Handle->SPIConfig.SPI_SSI << SPI_CR1_SSI);

	SPI_Handle->pSPIx->SPI_CR1=tempReg;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx==SPI1){
		SPI1_REG_RESET();
	}else if(pSPIx==SPI2){
		SPI2_REG_RESET();
	}else if(pSPIx==SPI3){
		SPI3_REG_RESET();
	}else if(pSPIx==SPI4){
		SPI4_REG_RESET();
	}else if(pSPIx==SPI5){
		SPI5_REG_RESET();
	}else if(pSPIx==SPI6){
		SPI6_REG_RESET();
	}
}

/*
 * Data Send and Receive
 * */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint8_t flagName){
	if(pSPIx->SPI_SR & flagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
//this is blocking call (with while loop)
void SPI_SendData(SPI_Handle_t *SPI_handle,uint8_t *pTxBuffer,uint32_t len){
	while(len>0){
		while(SPI_GetFlagStatus(SPI_handle->pSPIx,SPI_TXE_FLAG)==FLAG_RESET);
		if(SPI_handle->SPIConfig.SPI_DFF==SPI_DFF_8BITS){
			SPI2->SPI_DR=*pTxBuffer;
			pTxBuffer++;
			len--;
		}else if(SPI_handle->SPIConfig.SPI_DFF==SPI_DFF_16BITS){
			SPI_handle->pSPIx->SPI_DR=*((uint16_t*)pTxBuffer);
			(uint16_t*)pTxBuffer++;
			len--;
			len--;
		}
	}
}
void *SPI_ReceiveData(SPI_Handle_t *SPI_handle,uint8_t *pRxBuffer,uint32_t len){
	while(len!=0){
			while(SPI_GetFlagStatus(SPI_handle->pSPIx,SPI_RXNE_FLAG)==FLAG_RESET);
			if(SPI_handle->SPIConfig.SPI_DFF==SPI_DFF_8BITS){
				*pRxBuffer=SPI_handle->pSPIx->SPI_DR;
				pRxBuffer++;
				len--;
			}else if(SPI_handle->SPIConfig.SPI_DFF==SPI_DFF_16BITS){
				*((uint16_t*)pRxBuffer)=SPI_handle->pSPIx->SPI_DR;
				(uint16_t*)pRxBuffer++;
				len--;
				len--;
			}
	}
}
uint8_t SPI_SendDataIT(SPI_Handle_t *SPI_handle,uint8_t *pTxBuffer,uint32_t len){
	uint8_t state = SPI_handle->TxState;
	if(state != SPI_BUSY_IN_TX){
		SPI_handle->pTxbuffer=pTxBuffer;
		SPI_handle->TxLen=len;

		SPI_handle->TxState=SPI_BUSY_IN_TX;
		SPI_handle->pSPIx->SPI_CR2 |= (1<<SPI_CR2_TXEIE);
	}
	return state;
}
uint8_t *SPI_ReceiveDataIT(SPI_Handle_t *SPI_handle,uint8_t *pRxBuffer,uint32_t len){
	uint8_t state = SPI_handle->RxState;
		if(state != SPI_BUSY_IN_RX){
			SPI_handle->pRxbuffer=pRxBuffer;
			SPI_handle->RxLen=len;

			SPI_handle->RxState=SPI_BUSY_IN_RX;
			SPI_handle->pSPIx->SPI_CR2 |= (1<<SPI_CR2_RXNEIE);
		}
		return state;
}

/*
 * IRQ Configuration and ISR handling
 * */
static void spi_txe_interrupt_handle(SPI_Handle_t *SPI_handle){
	// check the DFF bit in CR1
		if( (SPI_handle->pSPIx->SPI_CR1 & ( 1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load the data in to the DR
			*((uint16_t*)SPI_handle->pTxbuffer)=SPI_handle->pSPIx->SPI_DR ;
			SPI_handle->TxLen--;
			SPI_handle->TxLen--;
			(uint16_t*)SPI_handle->pTxbuffer++;
		}else
		{
			//8 bit DFF
			SPI_handle->pSPIx->SPI_DR =   *SPI_handle->pTxbuffer;
			SPI_handle->TxLen--;
			SPI_handle->pTxbuffer++;
		}

		if(! SPI_handle->TxLen)
		{
			//TxLen is zero , so close the spi transmission and inform the application that
			//TX is over.

			//this prevents interrupts from setting up of TXE flag
			SPI_CloseTransmisson(SPI_handle);
			SPI_ApplicationEventCallback(SPI_handle,SPI_EVENT_TX_CMPLT);
		}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *SPI_handle){
	if( (SPI_handle->pSPIx->SPI_CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF
				//1. load the data in to the DR
				*((uint16_t*)SPI_handle->pRxbuffer)=(uint16_t)SPI_handle->pSPIx->SPI_DR;
				SPI_handle->RxLen--;
				SPI_handle->RxLen--;
				SPI_handle->pRxbuffer++;
				SPI_handle->pRxbuffer++;
			}else
			{
				//8 bit DFF
				*(SPI_handle->pTxbuffer)=(uint8_t)SPI_handle->pSPIx->SPI_DR;
				SPI_handle->RxLen--;
				SPI_handle->pRxbuffer++;
			}

			if(! SPI_handle->RxLen)
			{
				//TxLen is zero , so close the spi transmission and inform the application that
				//TX is over.

				//this prevents interrupts from setting up of TXE flag
				SPI_CloseTransmisson(SPI_handle);
				SPI_ApplicationEventCallback(SPI_handle,SPI_EVENT_RX_CMPLT);
			}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *SPI_handle){
	uint8_t temp;
		//1. clear the ovr flag
		if(SPI_handle->TxState != SPI_BUSY_IN_TX)
		{
			temp = SPI_handle->pSPIx->SPI_DR;
			temp = SPI_handle->pSPIx->SPI_SR;
		}
		(void)temp;
		//2. inform the application
		SPI_ApplicationEventCallback(SPI_handle,SPI_EVENT_OVR_ERR);
}
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxbuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxbuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;

}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}
void SPI_IRQHandling(SPI_Handle_t SPI_handle){

	uint8_t temp1 , temp2;
	//first lets check for TXE

	temp1 = SPI_handle.pSPIx->SPI_SR & ( 1 << SPI_SR_TXE);
	temp2 = SPI_handle.pSPIx->SPI_CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(&SPI_handle);
	}

	// check for RXNE
	temp1 = SPI_handle.pSPIx->SPI_SR & ( 1 << SPI_SR_RXNE);
	temp2 = SPI_handle.pSPIx->SPI_CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(&SPI_handle);
	}

	// check for ovr flag
	temp1 = SPI_handle.pSPIx->SPI_SR & ( 1 << SPI_SR_OVR);
	temp2 = SPI_handle.pSPIx->SPI_CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(&SPI_handle);
	}

}
//////////
/*
 *Other functions
 * */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi){
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
void SPI_SPEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi){
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}else {
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
