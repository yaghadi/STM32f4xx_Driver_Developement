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
void delay(void){
	for(uint32_t i=0;i<200000/2;i++);
}
void SPI2_GPIOInit(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_AF;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_PIN_OP_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PINR_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=GPIO_PIN_AF5;


	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
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
	pSPI2Handle->SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV8;
	pSPI2Handle->SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	pSPI2Handle->SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	pSPI2Handle->SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	pSPI2Handle->SPIConfig.SPI_SSM=SPI_SSM_DI;
	pSPI2Handle->SPIConfig.SPI_SSI=SPI_SSI_DI;

	SPI_Init(pSPI2Handle);
}
void GPIOBtn_Init(void){
	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx=GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PINR_NO_PUPD;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_PClk_Control(GPIOA,ENABLE);
	GPIO_Init(&GpioBtn);

}
int main(){
	SPI_Handle_t SPI2Handle;
	char *text="Hello Yassine";
	SPI2_GPIOInit();
	GPIOBtn_Init();
	SPI2Init(&SPI2Handle);
	SPI_SSOEConfig(SPI2Handle.pSPIx,ENABLE);
	while(1){
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));
		delay();
		SPI_SPEConfig(SPI2Handle.pSPIx, ENABLE);
		uint8_t dataLength =strlen(text);
		SPI_SendData(&SPI2Handle, &dataLength,1);
		SPI_SendData(&SPI2Handle,(uint8_t*)text,strlen(text));
		while(SPI_GetFlagStatus(SPI2Handle.pSPIx,SPI_BUSY_FLAG));
		SPI_SPEConfig(SPI2Handle.pSPIx, DISABLE);

	}
	return 0;
}

