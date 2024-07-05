/*
 * 001_SPI_TX.c
 *
 *  Created on: Apr 15, 2024
 *      Author: PurplE
 */
#include "STM32f407xx.h"
#include <string.h>
#include <stdint.h>
/*SPI2
 * PB15: SP2_MOSI
 * PB14: SP2_MISO
 * PB13: SP2_SCLK
 * PB12: SP2_NSS
 * AF mode :5
 * */
void SPI2_GPIOInit(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_AF;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_PIN_OP_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PINR_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=GPIO_PIN_AF5;


//	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
//	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

}
void SPI2Init(SPI_Handle_t *pSPI2Handle){
	pSPI2Handle->pSPIx=SPI2;

	pSPI2Handle->SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	pSPI2Handle->SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	pSPI2Handle->SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV2;
	pSPI2Handle->SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	pSPI2Handle->SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	pSPI2Handle->SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	pSPI2Handle->SPIConfig.SPI_SSM=SPI_SSM_EN;
	pSPI2Handle->SPIConfig.SPI_SSI=SPI_SSI_EN;

	SPI_Init(pSPI2Handle);


}
int main(){
	SPI_Handle_t SPI2Handle;
	char *text="Hello Yassine";
	SPI2_GPIOInit();
	SPI2Init(&SPI2Handle);
	SPI_Enable(SPI2Handle.pSPIx, ENABLE);
	SPI_SendData(&SPI2Handle,(uint8_t*)text,strlen(text));
	while(!SPI_GetFlagStatus(SPI2Handle.pSPIx,SPI_BUSY_FLAG));
			SPI_SPEConfig(SPI2Handle.pSPIx, DISABLE);
	while(1);
	return 0;
}

