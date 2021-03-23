/*
 * stm32l4xx.h
 *
 *  Created on: Aug 29, 2020
 *      Author: Tilak Gupta
 */

#ifndef INC_STM32L4XX_H_
#define INC_STM32L4XX_H_

#include <stdint.h>

uint32_t SystemCoreClock;

#define __vo volatile

#define ENABLE 		 	1
#define DISABLE		 	0
#define SET 		 	ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
/*
 * Base Address - FLASH, SRAM, ROM
 */

#define SET_BIT(reg, bitPos)   ( (reg) |=  (0x1 << (bitPos)) )
#define CLEAR_BIT(reg, bitPos) ( (reg) &= ~(0x1 << (bitPos)) )
#define READ_BIT(reg, bitPos)  ( (reg) &   (0x1 << (bitPos)) )

/* ----------------------- PROCESSOR SPECIFIC DETAILS -----------------------------*/
#define NO_PR_BITS_IMPLEMENTED 	4

#define NVIC_ISER0_ADDR		((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1_ADDR		((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2_ADDR		((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3_ADDR		((__vo uint32_t *)0xE000E10C)

#define NVIC_ICER0_ADDR		((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1_ADDR		((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2_ADDR		((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3_ADDR		((__vo uint32_t *)0xE000E18C)

#define NVIC_IPR_BASEADDR	((__vo uint32_t *)0xE000E400)

#define NVIC_ISER_BIT_WIDTH			32
#define NVIC_PRI_FIELD_PER_PRI_REG	4
#define NVIC_PRI_FIELD_BIT_WIDTH	8

void Arm_IRQConfig(uint8_t IRQNum, uint8_t EnorDi);
void Arm_PriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);

/* ----------------------- MEMORY BASE ADDRESSES -----------------------------*/
#define FLASH_BASEADDR			0x08000000U  // Base address of FLASH
#define SRAM1_BASEADDR    		0x20000000U  // Base address of SRAM1
#define SRAM2_BASEADDR			0x10000000U  // Base address of SRAM2
#define ROM_BASEADDR			0x1FFF0000U  // Base address of ROM

/* ----------------------- PERIPHERAL BUS BASE ADDRESSES -----------------------------*/
/*
 * AHBx and APBx Bus Peripheral Base Addresses
 */

#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR 		0x40000000U
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x48000000U


/* ----------------------- PERIPHERAL BASE ADDRESSES -----------------------------*/
/*
 * Base addresses for peripherals present on AHB2 Bus
 * TO DO: Complete for all other peripherals
 */

#define GPIOA_BASEADDR			(AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB2PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB2PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB2PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB2PERIPH_BASEADDR + 0x1C00)
#define ADC1_BASEADDR			((AHB2PERIPH_BASEADDR) + 0X10040000UL)
#define ADC2_BASEADDR			((AHB2PERIPH_BASEADDR) + 0X10040100UL)
#define ADC3_BASEADDR			((AHB2PERIPH_BASEADDR) + 0X10040200UL)
#define ADC_COMMON_BASEADDR     ((AHB2PERIPH_BASEADDR) + 0X10040300UL)

/*
 * Base addresses for peripherals present on AHB1 Bus
 * TO DO: Complete for all other peripherals
 */
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)

/*
 * Base addresses for peripherals present on APB2 Bus
 * TO DO: Complete for all other peripherals
 */

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x0400)

#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x0000)

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses for peripherals present on APB1 Bus
 * TO DO: Complete for all other peripherals
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)



/*
 * Peripheral's Register Structures
 */

typedef struct
{
	__vo uint32_t ISR;
	__vo uint32_t IER;
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t RESERVED0;
	__vo uint32_t SMPR1;
	__vo uint32_t SMPR2;
	__vo uint32_t RESERVED1;
	__vo uint32_t TR1;
	__vo uint32_t TR2;
	__vo uint32_t TR3;
	__vo uint32_t RESERVED2;
	__vo uint32_t SQR1;
	__vo uint32_t SQR2;
	__vo uint32_t SQR3;
	__vo uint32_t SQR4;
	__vo uint32_t DR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t JSQR;
	__vo uint32_t RESERVED4[4];
	__vo uint32_t OFR1;
	__vo uint32_t OFR2;
	__vo uint32_t OFR3;
	__vo uint32_t OFR4;
	__vo uint32_t RESERVED5[4];
	__vo uint32_t JDR1;
	__vo uint32_t JDR2;
	__vo uint32_t JDR3;
	__vo uint32_t JDR4;
	__vo uint32_t RESERVED6[4];
	__vo uint32_t AWD2CR;
	__vo uint32_t AWD3CR;
	__vo uint32_t RESERVED7[2];
	__vo uint32_t DIFSEL;
	__vo uint32_t CALFACT;
} ADC_RegDef_t;

typedef struct
{
	__vo uint32_t CSR;
	     uint32_t RESERVED0;
	__vo uint32_t CCR;
	__vo uint32_t CDR;

} ADC_CommonRegDef_t;

typedef struct
{
	__vo uint32_t MODER;		/* Configure output mode				Address offset: 0x00 */
	__vo uint32_t OTYPER;		/* Output type							Address offset: 0x04 */
	__vo uint32_t OSPEEDR;		/* Output speed 						Address offset: 0x08 */
	__vo uint32_t PUPDR;		/* Pull-up/pull-down register 			Address offset: 0x0C */
	__vo uint32_t IDR;			/* Input data register					Address offset: 0x10 */
	__vo uint32_t ODR;			/* Output data register					Address offset: 0x14 */
	__vo uint32_t BSRR;			/* Bit set/reset register 				Address offset:	0x18 */
	__vo uint32_t LCKR;			/* Configuration lock register			Address offset: 0x1C */
	__vo uint32_t AFRL;			/* Alternate function low register  	Address offset:	0x20 */
	__vo uint32_t AFRH;			/* Alternate function high register 	Address offset: 0x24 */
	__vo uint32_t BRR;			/* Bit reset register 					Address offset: 0x28 */
	__vo uint32_t ASCR;			/* Analog switch control register		Address offset:	0x2C */

} GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;			/* Clock control register						Address offset: 0x00 */
	__vo uint32_t ICSCR;		/* Internal clock sources control register		Address offset: 0x04 */
	__vo uint32_t CFGR;			/* Clock configuration register					Address offset: 0x08 */
	__vo uint32_t PLLCFGR;		/* PLL configuration register					Address offset: 0x0C */
	__vo uint32_t PLLSAI1CFGR;	/* PLLSAI1 configuration register				Address offset: 0x10 */
	__vo uint32_t PLLSAI2CFGR;	/* PLLSAI2 configuration register				Address offset: 0x14 */
	__vo uint32_t CIER;			/* Clock interrupt enable register				Address offset: 0x18 */
	__vo uint32_t CIFR;			/* Clock interrupt flag register				Address offset: 0x1C */
	__vo uint32_t CICR;			/* Clock interrupt clear register				Address offset: 0x20 */
	uint32_t RESERVED0;			/* 												Address offset: 0x24 */
	__vo uint32_t AHB1RSTR;		/* AHB1 peripheral reset register 				Address offset: 0x28 */
	__vo uint32_t AHB2RSTR;		/* AHB2 peripheral reset register				Address offset: 0x2C */
	__vo uint32_t AHB3RSTR;		/* AHB3 peripheral reset register				Address offset: 0x30 */
	uint32_t RESERVED1;			/* 												Address offset: 0x34 */
	__vo uint32_t APB1RSTR1;	/* APB1 peripheral reset register 1 			Address offset: 0x38 */
	__vo uint32_t APB1RSTR2;	/* APB1 peripheral reset register 2 			Address offset: 0x3C */
	__vo uint32_t APB2RSTR;		/* APB2 peripheral reset register				Address offset: 0x40 */
	uint32_t RESERVED2;			/* 												Address offset: 0x44 */
	__vo uint32_t AHB1ENR;		/* AHB1 peripheral clock enable register		Address offset: 0x48 */
	__vo uint32_t AHB2ENR;		/* AHB2 peripheral clock enable register		Address offset: 0x4C */
	__vo uint32_t AHB3ENR;		/* AHB3 peripheral clock enable register							Address offset: 0x50 */
	uint32_t RESERVED3;			/* 																	Address offset: 0x54 */
	__vo uint32_t APB1ENR1;		/* APB1 peripheral clock enable register 							Address offset: 0x58 */
	__vo uint32_t APB1ENR2;		/* APB1 peripheral clock enable register 2 							Address offset: 0x5C */
	__vo uint32_t APB2ENR;		/* APB2 peripheral clock enable register 							Address offset: 0x60 */
	uint32_t RESERVED4;			/* 																	Address offset: 0x64 */
	__vo uint32_t AHB1SMENR;	/* AHB1 peripheral clocks enable in Sleep and Stop modes register 			Address offset: 0x68 */
	__vo uint32_t AHB2SMENR;	/* AHB2 peripheral clocks enable in Sleep and Stop modes register 			Address offset: 0x6C */
	__vo uint32_t AHB3SMENR;	/* AHB3 peripheral clocks enable in Sleep and Stop modes register 			Address offset: 0x70 */
	uint32_t RESERVED5;			/* 																			Address offset: 0x74 */
	__vo uint32_t APB1SMENR1;	/* APB1 peripheral clocks enable in Sleep and Stop modes register 1			Address offset: 0x78 */
	__vo uint32_t APB1SMENR2;	/* APB1 peripheral clocks enable in Sleep and Stop modes register 2 		Address offset: 0x7C */
	__vo uint32_t APB2SMENR;	/* APB2 peripheral clocks enable in Sleep and Stop modes register 			Address offset: 0x80 */
	uint32_t RESERVED6;			/* 																			Address offset: 0x84 */
	__vo uint32_t CCIPR;		/* Peripherals independent clock configuration register						Address offset: 0x88 */
	__vo uint32_t BDCR;			/* Backup domain control register									Address offset: 0x90 */
	__vo uint32_t CSR;			/* Control/status register											Address offset: 0x94 */
} RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR1;
	__vo uint32_t EMR1;
	__vo uint32_t RTSR1;
	__vo uint32_t FTSR1;
	__vo uint32_t SWIER1;
	__vo uint32_t PR1;
	uint32_t RESERVED;
	__vo uint32_t IMR2;
	__vo uint32_t EMR2;
	__vo uint32_t RTSR2;
	__vo uint32_t FTSR2;
	__vo uint32_t SWIER2;
	__vo uint32_t PR2;

} EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t CFGR1;
	__vo uint32_t EXTICR[4];
	__vo uint32_t SCSR;
	__vo uint32_t CFGR2;
	__vo uint32_t SWPR;
	__vo uint32_t SKR;
} SYSCFG_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
} SPI_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t TIMINGR;
	__vo uint32_t TIMEOUTR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t PECR;
	__vo uint32_t RXDR;
	__vo uint32_t TXDR;
} I2C_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t BRR;
	__vo uint32_t GTPR;
	__vo uint32_t RTOR;
	__vo uint32_t RQR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t RDR;
	__vo uint32_t TDR;
} USART_RegDef_t;

/*
 * Peripheral definitions
 */

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define ADC1        ((ADC_RegDef_t *)ADC1_BASEADDR)
#define ADC2        ((ADC_RegDef_t *)ADC2_BASEADDR)
#define ADC3        ((ADC_RegDeg_t *)ADC3_BASEADDR)
#define ADC_Common  ((ADC_CommonRegDef_t *)ADC_COMMON_BASEADDR)


#define RCC	((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1	((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t *)SPI3_BASEADDR)

#define I2C1	((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3	((I2C_RegDef_t *)I2C3_BASEADDR)


#define USART1	((USART_RegDef_t *)USART1_BASEADDR)
#define USART2  ((USART_RegDef_t *)USART2_BASEADDR)
#define USART3  ((USART_RegDef_t *)USART3_BASEADDR)
#define UART4	((USART_RegDef_t *)UART4_BASEADDR)
#define UART5	((USART_RegDef_t *)UART5_BASEADDR)



/* ----------------------- PERIPHERAL CLOCK ENABLE MACROS -----------------------------*/

/*
 * Clock Enable Macros for GPIOx
 */

#define GPIOA_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 0 ) )    /* GPIO Port A Peripheral Enable Clock */
#define GPIOB_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 1 ) )	  /* GPIO Port B Peripheral Enable Clock */
#define GPIOC_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 2 ) )	  /* GPIO Port C Peripheral Enable Clock */
#define GPIOD_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 3 ) )	  /* GPIO Port D Peripheral Enable Clock */
#define GPIOE_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 4 ) )	  /* GPIO Port E Peripheral Enable Clock */
#define GPIOF_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 5 ) )	  /* GPIO Port F Peripheral Enable Clock */
#define GPIOG_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 6 ) )	  /* GPIO Port G Peripheral Enable Clock */
#define GPIOH_PCLK_EN()		( RCC->AHB2ENR |= ( 1 << 7 ) )	  /* GPIO Port H Peripheral Enable Clock */

#define ADC_PCLK_EN()       ( RCC->AHB2ENR |= ( 1 << 13) )    /* ADC Peripheral Enable Clock */


/*
 * Clock Enable Macros for UART/USART
 */

#define USART1_PCLK_EN() 	( RCC->APB2ENR  |= ( 1 << 14) )		/* USART1 Peripheral Enable Clock */
#define USART2_PCLK_EN() 	( RCC->APB1ENR1 |= ( 1 << 17) )		/* USART2 Peripheral Enable Clock */
#define USART3_PCLK_EN() 	( RCC->APB1ENR1 |= ( 1 << 18) )		/* USART3 Peripheral Enable Clock */
#define UART4_PCLK_EN() 	( RCC->APB1ENR1 |= ( 1 << 19) )		/* UART4 Peripheral Enable Clock  */
#define UART5_PCLK_EN() 	( RCC->APB1ENR1 |= ( 1 << 20) )		/* UART5 Peripheral Enable Clock  */

/*
 * Clock Enable Macros for I2C
 */

#define I2C1_PCLK_EN() 	( RCC->APB1ENR1 |= ( 1 << 21) )		/* I2C1 Peripheral Enable Clock */
#define I2C2_PCLK_EN() 	( RCC->APB1ENR1 |= ( 1 << 22) )		/* I2C2 Peripheral Enable Clock */
#define I2C3_PCLK_EN() 	( RCC->APB1ENR1 |= ( 1 << 23) )		/* I2C3 Peripheral Enable Clock */

/*
 * Clock Enable Macros for SPI
 */

#define SPI1_PCLK_EN()  ( RCC->APB2ENR |= (1 << 12) )		/* SPI1 Peripheral Enable Clock */
#define SPI2_PCLK_EN()  ( RCC->APB1ENR1 |= (1 << 14) )		/* SPI2 Peripheral Enable Clock */
#define SPI3_PCLK_EN()  ( RCC->APB1ENR1 |= (1 << 15) )		/* SPI3 Peripheral Enable Clock */

/*
 * Clock Enable Macros for SYSCFG
 */

#define SYSCFG_PCLK_EN() 	( RCC->APB2ENR |= (1 << 0))
/* ----------------------- PERIPHERAL CLOCK DISABLE MACROS -----------------------------*/

/*
 * Clock Disable Macros for GPIOx
 */

#define GPIOA_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 0 ) )   /* GPIO Port A Peripheral Disable Clock */
#define GPIOB_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 1 ) )	  /* GPIO Port B Peripheral Disable Clock */
#define GPIOC_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 2 ) )	  /* GPIO Port C Peripheral Disable Clock */
#define GPIOD_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 3 ) )	  /* GPIO Port D Peripheral Disable Clock */
#define GPIOE_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 4 ) )	  /* GPIO Port E Peripheral Disable Clock */
#define GPIOF_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 5 ) )	  /* GPIO Port F Peripheral Disable Clock */
#define GPIOG_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 6 ) )	  /* GPIO Port G Peripheral Disable Clock */
#define GPIOH_PCLK_DI()		( RCC->AHB2ENR &= ~( 1 << 7 ) )	  /* GPIO Port H Peripheral Disable Clock */

/*
 * Reset GPIOx Peripherals
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB2RSTR |= (0x1 << 0)); (RCC->AHB2RSTR &= ~(0x1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB2RSTR |= (0x1 << 1)); (RCC->AHB2RSTR &= ~(0x1 << 1)); }while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB2RSTR |= (0x1 << 2)); (RCC->AHB2RSTR &= ~(0x1 << 2)); }while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB2RSTR |= (0x1 << 3)); (RCC->AHB2RSTR &= ~(0x1 << 3)); }while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB2RSTR |= (0x1 << 4)); (RCC->AHB2RSTR &= ~(0x1 << 4)); }while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB2RSTR |= (0x1 << 5)); (RCC->AHB2RSTR &= ~(0x1 << 5)); }while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB2RSTR |= (0x1 << 6)); (RCC->AHB2RSTR &= ~(0x1 << 6)); }while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB2RSTR |= (0x1 << 7)); (RCC->AHB2RSTR &= ~(0x1 << 7)); }while(0)

/*
 * Reset SPIx Peripherals
 */

#define SPI1_REG_RESET() 	do{ (RCC->APB2RSTR |= (0x1 << 12)); (RCC->APB2RSTR &= ~(0x1 << 12)); }while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR1 |= (0x1 << 14)); (RCC->APB1RSTR1 &= ~(0x1 << 14)); }while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR1 |= (0x1 << 15)); (RCC->APB1RSTR1 &= ~(0x1 << 15)); }while(0)

/*
 * Reset UART/USART Peripherals
 */
#define USART1_REG_RESET()	do{ (RCC->APB2RSTR |= (0x1 << 14)); (RCC->APB2RSTR &= ~(0x1 << 14)); }while(0)
#define USART2_REG_RESET()	do{ (RCC->APB1RSTR1 |= (0x1 << 17)); (RCC->APB2RSTR &= ~(0x1 << 17)); }while(0)
#define USART3_REG_RESET()	do{ (RCC->APB1RSTR1 |= (0x1 << 18)); (RCC->APB2RSTR &= ~(0x1 << 18)); }while(0)
#define UART4_REG_RESET()	do{ (RCC->APB1RSTR1 |= (0x1 << 19)); (RCC->APB2RSTR &= ~(0x1 << 19)); }while(0)
#define UART5_REG_RESET()	do{ (RCC->APB1RSTR1 |= (0x1 << 20)); (RCC->APB2RSTR &= ~(0x1 << 20)); }while(0)

/*
 * Clock Disable Macros for UART/USART
 */

#define USART1_PCLK_DI() 	( RCC->APB2ENR  &= ~( 1 << 14) )		/* USART1 Peripheral Disable Clock */
#define USART2_PCLK_DI() 	( RCC->APB1ENR1 &= ~( 1 << 17) )		/* USART2 Peripheral Disable Clock */
#define USART3_PCLK_DI() 	( RCC->APB1ENR1 &= ~( 1 << 18) )		/* USART3 Peripheral Disable Clock */
#define UART4_PCLK_DI() 	( RCC->APB1ENR1 &= ~( 1 << 19) )		/* UART4 Peripheral Disable Clock  */
#define UART5_PCLK_DI() 	( RCC->APB1ENR1 &= ~( 1 << 20) )		/* UART5 Peripheral Disable Clock  */


/*
 * Clock Disable Macros for I2C
 */

#define I2C1_PCLK_DI() 	( RCC->APB1ENR1 &= ~( 1 << 21) )		/* I2C1 Peripheral Disable Clock */
#define I2C2_PCLK_DI() 	( RCC->APB1ENR1 &= ~( 1 << 22) )		/* I2C2 Peripheral Disable Clock */
#define I2C3_PCLK_DI() 	( RCC->APB1ENR1 &= ~( 1 << 23) )		/* I2C3 Peripheral Disable Clock */


/*
 * Clock Disable Macros for SPI
 */

#define SPI1_PCLK_DI()  ( RCC->APB2ENR  &= ~(1 << 12) )		/* SPI1 Peripheral Disable Clock */
#define SPI2_PCLK_DI()  ( RCC->APB1ENR1 &= ~(1 << 14) )		/* SPI2 Peripheral Disable Clock */
#define SPI3_PCLK_DI()  ( RCC->APB1ENR1 &= ~(1 << 15) )		/* SPI3 Peripheral Disable Clock */

/* ----------------------- NVIC Stuff -----------------------------*/

#define EXTI0_IRQ_N0		6
#define EXTI1_IRQ_N0		7
#define EXTI2_IRQ_N0		8
#define EXTI3_IRQ_N0		9
#define EXTI4_IRQ_N0		10
#define EXTI5_9_IRQ_NO		23
#define SPI1_IRQ_NO			35
#define SPI2_IRQ_NO			36
#define EXTI10_15_IRQ_NO	40
#define SPI3_IRQ_NO			51

#endif /* INC_STM32L4XX_H_ */
