/*
 * STM32f407xx.h
 *
 *  Created on: Apr 7, 2024
 *      Author: PurplE
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>
#include <stddef.h>
#define __vo volatile
#define __weak __attribute__((weak))
//ARM M4 NVIC Specific Registers
#define NVIC_ISER0			(*(__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			(*(__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			(*(__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			(*(__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0			(*(__vo uint32_t*)0xE000E180)
#define NVIC_ICER1			(*(__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			(*(__vo uint32_t*)0xE000E188)
#define NVIC_ICER3			(*(__vo uint32_t*)0xE000E18C)

#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)

#define NO_BITS_NOT_IMPLEM		4
//Base Addresses of SRAM , FLASH and ROM

#define FLASH_BASE_ADDRESS   0X08000000U
#define SRAM1_BASE_ADDRESS   0X20000000U
#define SRAM2_BASE_ADDESS    0x2001C000U
#define ROM_BASE_ADDRESS     0x1FFF0000U

//Base addresses of AHBx , APBx

#define PERIPHERAL_BASE      0X40000000U
#define APB1_PERIPH_BASE     PERIPHERAL_BASE
#define APB2_PERIPH_BASE     0X40010000U
#define AHB1_PERIPH_BASE     0X40020000U
#define AHB2_PERIPH_BASE     0X50000000U

//Defining of Base Addresses hanging out of AHB1 BUS

#define GPIOA_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0x0000)
#define GPIOB_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0X0400)
#define GPIOC_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0X0800)
#define GPIOD_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0X0C00)
#define GPIOE_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0X1000)
#define GPIOF_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0X1400)
#define GPIOG_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0X1800)
#define GPIOH_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0X1C00)
#define GPIOI_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0X2000)
#define GPIOJ_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0X2400)
#define GPIOK_BASE_ADDRESS    (AHB1_PERIPH_BASE + 0X2800)
#define CRC_BASE_ADDRESS      (AHB1_PERIPH_BASE + 0X3000)
#define RCC_BASE_ADDRESS      (AHB1_PERIPH_BASE + 0X3800)

//Defining of Base Addresses hanging out of APB1 BUS
#define I2C1_BASE_ADDRESS      (APB1_PERIPH_BASE + 0X5400)
#define I2C2_BASE_ADDRESS      (APB1_PERIPH_BASE + 0X5800)
#define I2C3_BASE_ADDRESS      (APB1_PERIPH_BASE + 0X5C00)
#define SPI2_BASE_ADDRESS      (APB1_PERIPH_BASE + 0X3800)
#define SPI3_BASE_ADDRESS      (APB1_PERIPH_BASE + 0X3C00)
#define USART2_BASE_ADDRESS     (APB1_PERIPH_BASE + 0X4400)
#define USART3_BASE_ADDRESS     (APB1_PERIPH_BASE + 0X4800)
#define USART4_BASE_ADDRESS     (APB1_PERIPH_BASE + 0X4C00)
#define USART5_BASE_ADDRESS     (APB1_PERIPH_BASE + 0X5000)

//Defining of Base Addresses hanging out of APB2 BUS
#define SPI1_BASE_ADDRESS      (APB2_PERIPH_BASE + 0X3000)
#define SPI4_BASE_ADDRESS      (APB2_PERIPH_BASE + 0X3400)
#define SPI5_BASE_ADDRESS      (APB2_PERIPH_BASE + 0X5000)
#define SPI6_BASE_ADDRESS      (APB2_PERIPH_BASE + 0X5400)
#define USART1_BASE_ADDRESS     (APB2_PERIPH_BASE + 0X1000)
#define USART6_BASE_ADDRESS     (APB2_PERIPH_BASE + 0X1400)
#define EXTI_BASE_ADDRESS      (APB2_PERIPH_BASE + 0X3C00)
#define SYSCFG_BASE_ADDRESS    (APB2_PERIPH_BASE + 0X3800)


//portCode implementation
#define portCode(x)				((x==GPIOA) ? 0:\
								(x==GPIOB) ? 1:\
								(x==GPIOC) ? 2:\
								(x==GPIOD) ? 3:\
								(x==GPIOE) ? 4:\
								(x==GPIOF) ? 5:\
								(x==GPIOG) ? 6:\
								(x==GPIOH) ? 7:\
								(x==GPIOI) ? 8:0)
/////////////////////////

/*IRQ(Interrupt Request)numbers
 * */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI_5_9 	23
#define IRQ_NO_EXTI_10_15 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
//GPIO Register Definition
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];       //AFR[0] is AFRL and AFR[1] is AFRH
}GPIO_RegDef_t;

//EXTI Register Definition
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMP;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

// SYSCFG Register Definition
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t reserved[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;
//SPI Register Definition
typedef struct{
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;
}SPI_RegDef_t;
//RCC Register Definition
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t reserved0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	 uint32_t Reserved1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	 uint32_t Reserved2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	 uint32_t Reserved3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t Reserved4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	 uint32_t Reserved5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	 uint32_t Reserved6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;
/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;
//Peripheral Description
#define GPIOA          ((GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)
#define GPIOB          ((GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)
#define GPIOC          ((GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)
#define GPIOD          ((GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)
#define GPIOE          ((GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)
#define GPIOF          ((GPIO_RegDef_t*)GPIOF_BASE_ADDRESS)
#define GPIOG          ((GPIO_RegDef_t*)GPIOG_BASE_ADDRESS)
#define GPIOH          ((GPIO_RegDef_t*)GPIOH_BASE_ADDRESS)
#define GPIOI          ((GPIO_RegDef_t*)GPIOI_BASE_ADDRESS)
#define GPIOJ          ((GPIO_RegDef_t*)GPIOJ_BASE_ADDRESS)
#define GPIOK          ((GPIO_RegDef_t*)GPIOK_BASE_ADDRESS)
#define RCC			   ((RCC_RegDef_t*)RCC_BASE_ADDRESS)
#define EXTI			((EXTI_RegDef_t*)EXTI_BASE_ADDRESS)
#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDRESS)
#define SPI1			((SPI_RegDef_t*)SPI1_BASE_ADDRESS)
#define SPI2			((SPI_RegDef_t*)SPI2_BASE_ADDRESS)
#define SPI3			((SPI_RegDef_t*)SPI3_BASE_ADDRESS)
#define SPI4			((SPI_RegDef_t*)SPI4_BASE_ADDRESS)
#define SPI5			((SPI_RegDef_t*)SPI5_BASE_ADDRESS)
#define SPI6			((SPI_RegDef_t*)SPI6_BASE_ADDRESS)
#define I2C1			((I2C_RegDef_t*)I2C1_BASE_ADDRESS)
#define I2C2			((I2C_RegDef_t*)I2C2_BASE_ADDRESS)
#define I2C3			((I2C_RegDef_t*)I2C3_BASE_ADDRESS)
#define USART1			((USART_RegDef_t*)USART1_BASE_ADDRESS)
#define USART2			((USART_RegDef_t*)USART2_BASE_ADDRESS)
#define USART3			((USART_RegDef_t*)USART3_BASE_ADDRESS)
#define USART4			((USART_RegDef_t*)USART4_BASE_ADDRESS)
#define USART5			((USART_RegDef_t*)USART5_BASE_ADDRESS)
#define USART6			((USART_RegDef_t*)USART6_BASE_ADDRESS)
//Clock Enable Macros for GPIOx Peripheral
#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()			(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()			(RCC->AHB1ENR |= (1 << 10))

//Clock Enable Macros for I2Cx Peripheral
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23))
//Clock Enable Macros for SPIx Peripheral
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()			(RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()			(RCC->APB2ENR |= (1 << 21))

//Clock ENable Macros for USART Peripheral
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))
//Clock Enable Macros for SYSCFG Peripheral
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))

/*Disable Clock Macros  */
//GPIO CLOCK Disabling
#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 7))
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 8))
#define GPIOJ_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 9))
#define GPIOK_PCLK_DI()			(RCC->AHB1ENR &= ~ (1 << 10))
//I2C CLOCK Disabling
#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~ (1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~ (1 << 22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~ (1 << 23))
//Clock Disable Macros for SPIx Peripheral
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~ (1 << 12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~ (1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~ (1 << 15))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~ (1 << 13))
#define SPI5_PCLK_DI()			(RCC->APB2ENR &= ~ (1 << 20))
#define SPI6_PCLK_DI()			(RCC->APB2ENR &= ~ (1 << 21))
//Clock Disable Macros for USART Peripheral
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~ (1 << 4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~ (1 << 17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~ (1 << 18))
#define USART4_PCLK_DI()		(RCC->APB1ENR &= ~ (1 << 19))
#define USART5_PCLK_DI()		(RCC->APB1ENR &= ~ (1 << 20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~ (1 << 5))
//Clock Enable Macros for SYSCFG Peripheral
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~ (1 << 14))

//GPIO Register Reset Macros
#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~ (1 << 0));}while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~ (1 << 1));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~ (1 << 2));}while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~ (1 << 3));}while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~ (1 << 4));}while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~ (1 << 5));}while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~ (1 << 6));}while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~ (1 << 7));}while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 8));	(RCC->AHB1RSTR &= ~ (1 << 8));}while(0)
#define GPIOJ_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 9));	(RCC->AHB1RSTR &= ~ (1 << 9));}while(0)
#define GPIOK_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &= ~ (1 << 10));}while(0)
///////////////////////////////
//SPI Register Reset Macros
#define SPI1_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~ (1 << 12));}while(0)
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~ (1 << 14));}while(0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~ (1 << 15));}while(0)
#define SPI4_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~ (1 << 14));}while(0)
#define SPI5_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~ (1 << 20));}while(0)
#define SPI6_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~ (1 << 21));}while(0)
////////////////////////////////////////////////////////////////////////////////////
//i2C Register Reset Macros
#define I2C1_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~ (1 << 21));}while(0)
#define I2C2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~ (1 << 22));}while(0)
#define I2C3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~ (1 << 23));}while(0)
////////////////////////////////////////////////////////////////////////////////////
//USART Register Reset Macros
#define USART1_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB1RSTR &= ~ (1 << 21));}while(0)
#define USART2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~ (1 << 22));}while(0)
#define USART3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~ (1 << 23));}while(0)
#define USART4_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~ (1 << 21));}while(0)
#define USART5_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~ (1 << 22));}while(0)
#define USART6_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB1RSTR &= ~ (1 << 23));}while(0)
////////////////////////////////////////////////////////////////////////////////////

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define FLAG_SET	SET
#define FLAG_RESET	RESET

////////////////////////////////////////
/*
 * Bit field definition for SPI Peripheral
 * */
///////////////////////////////////////////////
//CR1 : Control Register 1
#define SPI_CR1_CPHA 			0
#define SPI_CR1_CPOL 			1
#define SPI_CR1_MSTR 			2
#define SPI_CR1_BR 				3
#define SPI_CR1_SPE 			6
#define SPI_CR1_LSBFIRST 		7
#define SPI_CR1_SSI 			8
#define SPI_CR1_SSM 			9
#define SPI_CR1_RXONLY 			10
#define SPI_CR1_DFF 			11
#define SPI_CR1_CRCNEXT 		12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15
//CR2 : Control Register 2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
//#define SPI_CR2_RESERVED		3
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7
//SR : Status Register
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

////////////////////////////////////////
/*
 * Bit field definition for UASRT Peripheral
 * */
///////////////////////////////////////////////
/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
