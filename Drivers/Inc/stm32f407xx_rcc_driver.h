/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Jul 4, 2024
 *      Author: PurplE
 */


#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "STM32f407xx.h"


uint32_t RCC_GetPLLOutputClock(void);
uint32_t RCC_GetPCKL1Value(void);
uint32_t RCC_GetPCKL2Value(void);





#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
