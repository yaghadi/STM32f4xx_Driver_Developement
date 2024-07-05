/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Apr 7, 2024
 *      Author: PurplE
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_
#include <stdint.h>
#include "STM32f407xx.h"

/*GPIO pin configuration */
typedef struct
{
	uint8_t GPIO_PinNumber;   		/* Pin configuration possible cases :@GPIO_PIN_NO*/
	uint8_t GPIO_PinMode;			/* Pin configuration possible cases :@GPIO_PIN_MODE*/
	uint8_t GPIO_PinSpeed;			/* Pin configuration possible cases :@GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl;	/* Pin configuration possible cases :@GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOPType;			/* Pin configuration possible cases :@GPIO_PIN_OPT*/
	uint8_t GPIO_PinAltFunMode;		/* Pin configuration possible cases :@GPIO_PIN_AF*/
}GPIO_PinConfig_t;
////////////////////////


/*@GPIO_PIN_NO
 * HERE YOU CAN Find all possible cases for Pin Number
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15
////////////////



/*@GPIO_PIN_MODE
 * HERE YOU CAN Find all possible cases for Pin MODE
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_AF		2
#define GPIO_MODE_AN		3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_FRT	6
/////////////////


/*@GPIO_PIN_SPEED
 * HERE YOU CAN Find all possible cases for Pin Speed Configuration
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3
///////////////////////////////////////


/*@GPIO_PIN_PUPD
 * HERE YOU CAN Find all possible cases for Pin Pull-up and Pull-down Configuration
 */
#define GPIO_PINR_NO_PUPD		0
#define GPIO_PINR_PU				1
#define GPIO_PINR_PD				2
/////////////////////////////////////////


/*@GPIO_PIN_OPT
 * HERE YOU CAN Find all possible cases for Pin Speed Configuration
 */
#define GPIO_PIN_OP_PP		0
#define GPIO_PIN_OP_OD		1
//////////////////////////////////////////////////////


/*@GPIO_PIN_AFT
 * HERE YOU CAN Find all possible cases for alternate function Configuration
 */
#define GPIO_PIN_AF0		0
#define GPIO_PIN_AF1		1
#define GPIO_PIN_AF2		2
#define GPIO_PIN_AF3		3
#define GPIO_PIN_AF4		4
#define	GPIO_PIN_AF5		5
#define GPIO_PIN_AF6		6
#define GPIO_PIN_AF7		7
#define GPIO_PIN_AF8		8
#define GPIO_PIN_AF9		9
#define GPIO_PIN_AF10		10
#define	GPIO_PIN_AF11		11
#define GPIO_PIN_AF12		12
#define GPIO_PIN_AF13		13
#define GPIO_PIN_AF14		14
#define GPIO_PIN_AF15		15
/////////////////////////////////////////////////





/*GPIO Handle struct
 * */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/*
 * Init and De-init
 * */
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*                              SPI APIs supported by this driver
 * Peripheral clock setup
 * */
void GPIO_PClk_Control(GPIO_RegDef_t *pGPIOx,uint8_t EnOrDi);
/*
 * Data Send and Receive
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t pinNumber);
/*
 * IRQ Configuration and ISR handling
 * */
void GPIO_IRQConfing(uint8_t IRQNumber,uint8_t EnOrDi);
void GPIO_IRQPriorityConfing(uint8_t IRQNum,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNUmber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
