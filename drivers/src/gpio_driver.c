/*
 * stm32l47x_gpio_driver.c
 *
 *  Created on: Aug 31, 2020
 *      Author: Tilak Gupta
 */


#include <gpio_driver.h>

/*
 * GPIO Clock Control
 */

void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx ==  GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx ==  GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/*
 * GPIO Initialize and Deinitialize
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	uint32_t mask = 0;

	// 1. Set mode
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	mask = 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		pGPIOHandle->pGPIOx->MODER = (pGPIOHandle->pGPIOx->MODER & (~mask)) | temp;
	}
	else
	{
		temp = GPIO_MODE_IN << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//TODO: Make sure this is correct. Set the pin in input mode?
		pGPIOHandle->pGPIOx->MODER = (pGPIOHandle->pGPIOx->MODER & (~mask)) | temp;

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR1 |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear corresponding RTSR bit
			EXTI->RTSR1 &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR1 |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear FTSR bit
			EXTI->FTSR1 &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR1 |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure SYSCNFG - select GPIO Port for Interrupt Pin
		SYSCFG_PCLK_EN();
		int cfgrNum = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		int cfgrPos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG->EXTICR[cfgrNum] = (SYSCFG->EXTICR[cfgrNum] & ~(0xF << cfgrPos)) | (portCode << cfgrPos);

		// 3. Enable EXTI interrupt interrupt delivery using IMR
		EXTI->IMR1 |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	// 2. Set speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	mask = 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR = (pGPIOHandle->pGPIOx->OSPEEDR & (~mask)) | temp;

	// 3. Set PU/PD
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	mask = 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR = (pGPIOHandle->pGPIOx->PUPDR & (~mask)) | temp;

	// 4. Set OP type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	mask = 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER = (pGPIOHandle->pGPIOx->OTYPER &  (~mask)) | temp;

	// 5. Set ALT_FCN
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFCN)
	{
		// Choose high or low register depending on pin #
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
			mask = 0xF << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFRL = (pGPIOHandle->pGPIOx->AFRL & (~mask)) | temp;
		}
		else
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8)) );
			mask = 0xF << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8));
			pGPIOHandle->pGPIOx->AFRH = (pGPIOHandle->pGPIOx->AFRH & (~mask)) | temp;

		}
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) // how can we tell which GPIO port this is just by looking at address?
{
	//AHB2 reset register in RCC Peripheral
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx ==  GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


/*
 * GPIO Read and Write
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
}

uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)(pGPIOx->IDR & 0x0000FFFF);
}

void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
	{
		pGPIOx->ODR |= 0x1 << PinNumber;
	}
	else if(Value == RESET)
	{
		pGPIOx->ODR &= ~(0x1 << PinNumber);
	}
}

void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = (pGPIOx->ODR & (~0x0000FFFF)) | ((uint32_t)value);
}

void GPIO_WriteTogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (0x1 << PinNumber);
}



/*
 * IRQ configuration and handling
 */

void GPIO_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNum <= 31 && IRQNum >= 0)
		{
			(*NVIC_ISER0_ADDR) |=  (0x1 << (IRQNum % 32));
		}
		else if(IRQNum <= 63 && IRQNum >= 32)
		{
			(*NVIC_ISER1_ADDR) |=  (0x1 << (IRQNum % 32));
		}
		else if(IRQNum <= 95 && IRQNum >= 64)
		{
			(*NVIC_ISER2_ADDR) |=  (0x1 << (IRQNum % 32));
		}
		else if(IRQNum <= 127 && IRQNum >= 96)
		{
			(*NVIC_ISER3_ADDR) |=  (0x1 << (IRQNum % 32));
		}
	}
	else
	{
		if(IRQNum <= 31 && IRQNum >= 0)
		{
			(*NVIC_ICER0_ADDR) |=  (0x1 << (IRQNum % 32));
		}
		else if(IRQNum <= 63 && IRQNum >= 32)
		{
			(*NVIC_ICER1_ADDR) |=  (0x1 << (IRQNum % 32));
		}
		else if(IRQNum <= 95 && IRQNum >= 64)
		{
			(*NVIC_ICER2_ADDR) |=  (0x1 << (IRQNum % 32));
		}
		else if(IRQNum <= 127 && IRQNum >= 96)
		{
			(*NVIC_ICER3_ADDR) |=  (0x1 << (IRQNum % 32));
		}
	}
}

void GPIO_PriorityConfig(uint8_t IRQNum, uint32_t IRQPriority)
{
	uint8_t iprNum = IRQNum / 4;
	uint8_t iprPos = IRQNum % 4;

	(*(NVIC_IPR_BASEADDR + iprNum)) = ( (*(NVIC_IPR_BASEADDR + iprNum)) & ~(0xFF << (iprPos*8)) ) | ( IRQPriority << ((8 - NO_PR_BITS_IMPLEMENTED) + (iprPos*8)) );
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR1 & (1 << PinNumber))
	{
		// clear the PR Bit by writing a 1 (i know, it is counter-intuitive)
		EXTI->PR1 |= (1 << PinNumber);
	}
}

