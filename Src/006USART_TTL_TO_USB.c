/*
 * 006USART_TTL_TO_USB.c
 *
 *  Created on: Jul 5, 2024
 *      Author: PurplE
 */
#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"


char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart4_handle; //i'm using USART4 on the stm32f4 discovery

void USART4_Init(void)
{
	usart4_handle.pUSARTx = USART4;
	usart4_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart4_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart4_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart4_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart4_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart4_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart4_handle);
}

void USART4_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_PIN_OP_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PINR_NO_PUPD;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =8;

	//USART2 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_0;
	GPIO_Init(&usart_gpios);

	//USART2 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&usart_gpios);


}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PINR_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_PIN_OP_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PINR_NO_PUPD;

	//GPIO_PClk_Control(GPIOD,ENABLE);


	GPIO_Init(&GpioLed);

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

	GPIO_ButtonInit();

	USART4_GPIOInit();

    USART4_Init();

    USART_PeripheralControl(USART4,ENABLE);

    while(1)
    {
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) ){
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
			delay();
		}


		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		USART_SendData(&usart4_handle,(uint8_t*)msg,strlen(msg));

    }

	return 0;
}
