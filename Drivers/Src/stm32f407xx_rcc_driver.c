/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Jul 4, 2024
 *      Author: PurplE
 */
#include "stm32f407xx_rcc_driver.h"

uint32_t AHB_PreScalers[8]={2,4,8,16,64,128,256,512};
uint32_t APB1_PreScalers[4]={2,4,8,16};
uint32_t APB2_PreScalers[4]={2,4,8,16};

uint32_t RCC_GetPLLOutputClock(void){

	return 0;
}
uint32_t RCC_GetPCKL1Value(void){
	uint32_t pckl,SystemCKL;
	uint8_t cklsrc,ahbp,temp,apb1p;
	//Clock source extraction
	cklsrc = ((RCC->CFGR >> 2) & 0x3);
	if(cklsrc==0){
		SystemCKL=16000000;
	}else if(cklsrc==1){
		SystemCKL=8000000;
	}else if(cklsrc==2){
		SystemCKL=RCC_GetPLLOutputClock();
	}
	//AHB Prescaler extraction
	temp =((RCC->CFGR >> 4) & 0xF);
	if(temp < 8){
		ahbp=1;
	}else{
		ahbp=AHB_PreScalers[temp-8];
	}
	//APB1 Prescaler extraction
	temp =((RCC->CFGR >> 10) & 0x7);
	if(temp < 4){
		apb1p=1;
	}else{
		apb1p=APB1_PreScalers[temp-4];
	}

	pckl=((SystemCKL/ahbp)/apb1p);

	return pckl;
}
uint32_t RCC_GetPCKL2Value(void){
	uint32_t pckl,SystemCKL;
	uint8_t cklsrc,ahbp,temp,apb2p;
	//Clock source extraction
	cklsrc = ((RCC->CFGR >> 2) & 0x3);
	if(cklsrc==0){
		SystemCKL=16000000;
	}else if(cklsrc==1){
		SystemCKL=8000000;
	}else if(cklsrc==2){
		SystemCKL=RCC_GetPLLOutputClock();
	}
	//AHB Prescaler extraction
	temp =((RCC->CFGR >> 4) & 0xF);
	if(temp < 8){
		ahbp=1;
	}else{
		ahbp=AHB_PreScalers[temp-8];
	}
	//APB2 Prescaler extraction
	temp =((RCC->CFGR >> 13) & 0x7);
	if(temp < 4){
		apb2p=1;
	}else{
		apb2p=APB2_PreScalers[temp-4];
	}

	pckl=((SystemCKL/ahbp)/apb2p);

	return pckl;
}
