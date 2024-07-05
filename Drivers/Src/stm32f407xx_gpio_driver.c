/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Apr 7, 2024
 *      Author: PurplE
 */
#include "stm32f407xx_gpio_driver.h"


/*
 * GPIO Clock Enable API
 * */
void GPIO_PClk_Control(GPIO_RegDef_t *pGPIOx,uint8_t EnOrDi){
	if(EnOrDi){
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_EN();
		}
		if(pGPIOx==GPIOB){
					GPIOB_PCLK_EN();
		}
		if(pGPIOx==GPIOC){
					GPIOC_PCLK_EN();
		}
		if(pGPIOx==GPIOD){
					GPIOD_PCLK_EN();
		}
		if(pGPIOx==GPIOE){
					GPIOE_PCLK_EN();
		}
		if(pGPIOx==GPIOF){
					GPIOF_PCLK_EN();
		}
		if(pGPIOx==GPIOG){
					GPIOG_PCLK_EN();
		}
		if(pGPIOx==GPIOH){
					GPIOH_PCLK_EN();
		}
		if(pGPIOx==GPIOI){
					GPIOI_PCLK_EN();
		}
		if(pGPIOx==GPIOJ){
					GPIOJ_PCLK_EN();
		}
		if(pGPIOx==GPIOK){
					GPIOK_PCLK_EN();
		}
	}else{
		if(pGPIOx==GPIOA){
					GPIOA_PCLK_DI();
		}
		if(pGPIOx==GPIOB){
					GPIOB_PCLK_DI();
		}
		if(pGPIOx==GPIOC){
					GPIOC_PCLK_DI();
		}
		if(pGPIOx==GPIOD){
					GPIOD_PCLK_DI();
		}
		if(pGPIOx==GPIOE){
					GPIOE_PCLK_DI();
		}
		if(pGPIOx==GPIOF){
					GPIOF_PCLK_DI();
		}
		if(pGPIOx==GPIOG){
					GPIOG_PCLK_DI();
		}
		if(pGPIOx==GPIOH){
					GPIOH_PCLK_DI();
		}
		if(pGPIOx==GPIOI){
					GPIOI_PCLK_DI();
		}
		if(pGPIOx==GPIOJ){
					GPIOJ_PCLK_DI();
		}
		if(pGPIOx==GPIOK){
					GPIOK_PCLK_DI();
		}

	}
}
///////////////////////////////////////////////////////////////////////

////////////// Initialization of GPIO_Init ////////////////
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle){
	GPIO_PClk_Control(pGPIO_Handle->pGPIOx, ENABLE);

	uint32_t temp=0;
	//configure the mode of gpio pin
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode<GPIO_MODE_AN){
		temp=((pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber )));
		pGPIO_Handle->pGPIOx->MODER &= ~(0x3 << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER |= temp;
		temp=0;
	}else{
		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_FT){
			//setting FTSR;
			EXTI->FTSR |= (1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			//clearing RTSR;
			EXTI->RTSR &= ~(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_RT){
			EXTI->RTSR |= (1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			//clearing RTSR;
			EXTI->FTSR &= ~(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_FRT){
			EXTI->FTSR |= (1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			//clearing RTSR;
			EXTI->RTSR |= (1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//configure GPIO port selection in SYSCFG_EXITCR
		uint8_t temp1=pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2=pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber%4;
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]=(portCode(pGPIO_Handle->pGPIOx) << (temp2*4));
		//enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	}
	//configure the speed
	temp=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp;
	temp=0;

	//configure the pupd setting
	temp=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR |= temp;
	temp=0;
	//configure the optype
	temp=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType << (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OTYPER &= ~(0x1 << (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OTYPER |= temp;
	temp=0;
	//configure the alt functionality
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_AF){
		uint32_t temp1,temp2;
		temp1=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 8);
		temp2=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 8);
		pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(0xf << (4*temp2));
		pGPIO_Handle->pGPIOx->AFR[temp1] |= ((pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode) << (4* temp2));
	}

}
///////////////////////////////////////////////////////////////////////////////////////////
/*GPIO Reset api
 * */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx==GPIOA){
		GPIOA_REG_RESET();
	}
	if(pGPIOx==GPIOB){
		GPIOB_REG_RESET();
	}
	if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();
	}
	if(pGPIOx==GPIOD){
		GPIOD_REG_RESET();
	}
	if(pGPIOx==GPIOE){
		GPIOE_REG_RESET();
	}
	if(pGPIOx==GPIOF){
		GPIOF_REG_RESET();
	}
	if(pGPIOx==GPIOG){
		GPIOG_REG_RESET();
	}
	if(pGPIOx==GPIOH){
		GPIOH_REG_RESET();
	}
	if(pGPIOx==GPIOI){
		GPIOI_REG_RESET();
	}
	if(pGPIOx==GPIOJ){
		GPIOJ_REG_RESET();
	}
	if(pGPIOx==GPIOK){
		GPIOK_REG_RESET();
	}
}
//////////////////////////////////////////////////////////////////////
/*Function implementation to read input from pin
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber){
	return((pGPIOx->IDR >> pinNumber) & 0x00000001);

}
///////////////////////////////////////////////

/*Function implementation to read input from pin
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	return (uint16_t)pGPIOx->IDR;
}
/////////////////////////////////////////////////////


/*Function implementation to write to output pin
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber,uint8_t Value){
	if(Value){
		pGPIOx->ODR &= ~(0x1 << pinNumber);
		pGPIOx->ODR |= (uint32_t)(1 << pinNumber);
	}else{
		pGPIOx->ODR &= ~(uint32_t)(0 << pinNumber);
	}
}
/////////////////////////////////////////////////////


/*Function implementation to write to output Port
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value){
	pGPIOx->ODR &= ~(0xffffffff);
	pGPIOx->ODR |= (uint16_t)(Value);
}
/////////////////////////////////////////////////////

/*Function implementation to Toggle the specific output Pin
 * */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber){
	pGPIOx->ODR ^=(1 << pinNumber);
}
/////////////////////////////////////////////////////
/*Function implementation to Interrupt configuration
 * */
void GPIO_IRQConfing(uint8_t IRQNumber,uint8_t EnOrDi){
	if(EnOrDi){
		if(IRQNumber <31){
			NVIC_ISER0 |= (1 <<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64){
			NVIC_ISER1 |= (1 <<(IRQNumber%32));

		}else if (IRQNumber >63 && IRQNumber <96){
			NVIC_ISER2 |= (1 <<(IRQNumber%64));
		}
	}else{
		if(IRQNumber <31){
			NVIC_ICER0 |= (1 <<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64){
			NVIC_ICER1 |= (1 <<(IRQNumber%32));

		}else if (IRQNumber >63 && IRQNumber <96){
			NVIC_ICER2 |= (1 <<(IRQNumber%64));
		}
	}
}
/////////////////////////////////////////////////////
/*Function implementation for IRQHandling
 * */
void GPIO_IRQPriorityConfing(uint8_t IRQNum,uint8_t IRQPriority){
	uint8_t iprx=IRQNum/4;
	uint8_t iprx_section=IRQNum%8;
	uint16_t shiftAmount=(8*iprx_section)+(8-NO_BITS_NOT_IMPLEM);
	*(NVIC_PR_BASE_ADDR +(iprx)) |= (IRQPriority << shiftAmount);
}
/////////////////////////////////////////////////
/*Function implementation for handling Interrupt
 * */
void GPIO_IRQHandling(uint8_t pinNUmber){
	if(EXTI->PR & (1<<pinNUmber)){
		EXTI->PR |=(1 << pinNUmber);
	}
}
/////////////////////////////////////////////////////
