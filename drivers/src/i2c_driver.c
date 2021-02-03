/*
 * i2c_driver.c
 *
 *  Created on: September 10, 2020
 *      Author: Tilak Gupta
 */
#include "i2c_driver.h"

void I2C_PeriClkEnDi(I2C_Handle_t *pi2c, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		if(pi2c->pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pi2c->pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pi2c->pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pi2c->pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pi2c->pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pi2c->pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}

}

void I2C_Init(I2C_Handle_t *pi2c)
{
	// 1. Clear PE (I2C Peripheral Enable) Bit
	pi2c->pI2Cx->CR1 &= ~(0x1);

	// 2. Configure timing register - only supporting standard mode
	pi2c->pI2Cx->TIMINGR &= ~(0xF << 28); // Clear PRESC (PRESC = 0)

	pi2c->pI2Cx->TIMINGR &= ~(0xF << 20); // Clear SCLDEL
	pi2c->pI2Cx->TIMINGR |=  (0x4 << 20); // Set SCLDEL

	pi2c->pI2Cx->TIMINGR &= ~(0xF << 16); // Clear SDADEL
	pi2c->pI2Cx->TIMINGR |=  (0x2 << 16); // Set SDADEL

	pi2c->pI2Cx->TIMINGR &= ~(0xFF << 8); // Clear SCLH
	pi2c->pI2Cx->TIMINGR |=  (0xF << 8);  // Set SCLH

	pi2c->pI2Cx->TIMINGR &= ~(0xFF); // Clear SCLL
	pi2c->pI2Cx->TIMINGR |=  (0x13); // Set SCLL

	// 3. Configure device own address
	pi2c->pI2Cx->OAR1 &= ~(0x3ff);	  // reset OA1 Mode and address fields
	pi2c->pI2Cx->OAR1 |= ( ((uint32_t)pi2c->config.deviceAddress) << 1);	// write OA1 7 bit address
	pi2c->pI2Cx->OAR1 |= (0x1 << 15); // set OA1 Enable Flag. Must be set last

	// 4. Configure NOSTRETCH
	pi2c->pI2Cx->CR1 &= ~(0x1 << 17); // reset NOSTRETCH to allow clock stretching

	// 5. Set PE (I2C Peripheral Enable) bit
	pi2c->pI2Cx->CR1 |= (0x1);
}

void I2C_Deinit(I2C_Handle_t *pi2c)
{
	if(pi2c->pI2Cx == I2C1)
	{
		RCC->APB1RSTR1 |=  (1 << 21);
		RCC->APB1RSTR1 &= ~(1 << 21);
	}
	else if(pi2c->pI2Cx == I2C2)
	{
		RCC->APB1RSTR1 |=  (1 << 22);
		RCC->APB1RSTR1 &= ~(1 << 22);
	}
	else if(pi2c->pI2Cx == I2C3)
	{
		RCC->APB1RSTR1 |=  (1 << 23);
		RCC->APB1RSTR1 &= ~(1 << 23);
	}
}


void I2C_Master_TXRXPolling(I2C_Handle_t *pi2c)
{
	// 1. Addressing mode (7 bit or 10 bit) ADD10
	if(pi2c->commConfig.slaveAddMode == I2C_ADD_MODE_7)
	{
		pi2c->pI2Cx->CR2 &= ~(1 << 11); // clear ADD10 bit
	}
	else
	{
		while(1); // "Only 7-bit addressing is supported!");
	}

	// 2. Slave address to be sent SADD[9:0]
	pi2c->pI2Cx->CR2 &= ~(0xFF << 1);
	pi2c->pI2Cx->CR2 |= (pi2c->commConfig.slaveAdd << 1);

	// 3. Transfer Direction RD_WRN
	if(pi2c->commConfig.readNotWrite == 0)
	{
		pi2c->pI2Cx->CR2 &= ~(0x1 << 10);
	}
	else
	{
		pi2c->pI2Cx->CR2 |= (0x1 << 10);
	}

	// 4. HEAD10R Bit - Don't think I need to worry about this.

	// 5. # of bytes to be transferred NBYTES[7:0]
	if(pi2c->commConfig.buffLen > 255)
	{
		pi2c->pI2Cx->CR2 |= (0xFF << 16); // Set NBYTES to max value of 255
		pi2c->pI2Cx->CR2 |= (0x1 << 24);  // Set RELOAD bit
	}
	else
	{
		pi2c->pI2Cx->CR2 &= ~(0xFF << 16); 					   // Clear NBYTES field
		pi2c->pI2Cx->CR2 |= (pi2c->commConfig.buffLen << 16 ); // Set NBYTES
		pi2c->pI2Cx->CR2 &= ~(0x1 << 24);  					   // Reset RELOAD bit
	}


	// 6. Set AUTOEND bit
	pi2c->pI2Cx->CR2 |= (1 << 25);

	// 7. Set the start bit
	pi2c->pI2Cx->CR2 |= (1 << 13);

	// 8. Determine whether master communication with the slave was successful
	while( (pi2c->pI2Cx->CR2 & (1 << 13)) ); // wait for START bit to be cleared by HW

	// if Arbitration loss bit is set
	if(pi2c->pI2Cx->ISR & (1 << 9))
	{
		// clear ARLO flag and inform application
		pi2c->pI2Cx->ICR |= (1 << 9);
		I2C_AppEvCallback(I2C_EV_M_COMM_FAIL);
	}
	// else if NACKF bit is set
	else if(pi2c->pI2Cx->ISR & (1 << 4))
	{
		// clear NACKF bit and inform application
		pi2c->pI2Cx->ICR |= (1 << 4); // clear NACKF flag by setting NACKFCF
		I2C_AppEvCallback(I2C_EV_M_COMM_FAIL);
	}

	// 9. Depending on read or write, call appropriate function
	if(pi2c->commConfig.readNotWrite == 1)
	{
		I2C_MasterRXPolling(pi2c);
	}
	else
	{
		I2C_MasterTXPolling(pi2c);
	}
}

/*
 * I2C Master TX Data (Blocking Mode)
 */
static void I2C_MasterTXPolling(I2C_Handle_t *pi2c)
{
	uint8_t *pTX = pi2c->commConfig.pBuff;

	while(pi2c->commConfig.buffLen > 0)
	{

		// wait for TXISF or NACKF
		while( !(pi2c->pI2Cx->ISR & (1 << 1)) && !(pi2c->pI2Cx->ISR & (1 << 4)) );

		if(pi2c->pI2Cx->ISR & (1 << 4)) //if NACKF
		{
			I2C_AppEvCallback(I2C_EV_M_TX_FAIL);
			return;
		}

		if(pi2c->pI2Cx->ISR & (1 << 7)) // if TCR
		{
			if(pi2c->commConfig.buffLen > 255)
			{
				pi2c->pI2Cx->CR2 |= (0xFF << 16); // Set NBYTES to max value of 255
				pi2c->pI2Cx->CR2 |= (0x1 << 24);  // Set RELOAD bit
			}
			else
			{
				pi2c->pI2Cx->CR2 |= (pi2c->commConfig.buffLen << 16); // Set NBYTES
				pi2c->pI2Cx->CR2 &= ~(1 << 24); 	// Clear the RELOAD bit
			}
		}

		*((volatile uint8_t *)&pi2c->pI2Cx->TXDR) = *pTX;

		pTX++;
		pi2c->commConfig.buffLen--;
	}

}

/*
 * I2C Master RX Data (Blocking Mode)
 */
static void I2C_MasterRXPolling(I2C_Handle_t *pi2c)
{
	uint8_t *pRX = pi2c->commConfig.pBuff;

	while(pi2c->commConfig.buffLen > 0)
	{
		// wait for RXNE flag
		while( !(pi2c->pI2Cx->ISR & (1 << 2)) );

		*pRX = *((volatile uint8_t *)&pi2c->pI2Cx->RXDR);
		pRX++;
		pi2c->commConfig.buffLen--;

		if(pi2c->pI2Cx->ISR & (1 << 7)) // if TCR
		{
			if(pi2c->commConfig.buffLen > 255)
			{
				pi2c->pI2Cx->CR2 |= (0xFF << 16); // Set NBYTES to max value of 255
				pi2c->pI2Cx->CR2 |= (0x1 << 24);  // Set RELOAD bit
			}
			else
			{
				pi2c->pI2Cx->CR2 |= (pi2c->commConfig.buffLen << 16); // Set NBYTES
				pi2c->pI2Cx->CR2 &= ~(1 << 24); 	// Clear the RELOAD bit
			}
		}
	}
}




/*
 * I2C Interrupts
 */

uint8_t I2C_MasterTXRX_INT(I2C_Handle_t *pi2c, uint8_t buffLen, uint8_t *pBuff, uint8_t rnw)
{
	pi2c->pI2Cx->CR1 &= ~(0xFE << 0); // Clear all interrupt enable bits

	uint8_t state = pi2c->state;
	if(state != I2C_STATE_READY)
	{
		return pi2c->state;
	}
	pi2c->commConfig.buffLen = buffLen;
	pi2c->commConfig.pBuff = pBuff;
	pi2c->commConfig.readNotWrite = rnw;

	// 1. Addressing mode (7 bit or 10 bit) ADD10
	if(pi2c->commConfig.slaveAddMode == I2C_ADD_MODE_7)
	{
		pi2c->pI2Cx->CR2 &= ~(1 << 11); // clear ADD10 bit
	}
	else
	{
		while(1); // "Only 7-bit addressing is supported!");
	}

	// 2. Slave address to be sent SADD[9:0]
	pi2c->pI2Cx->CR2 &= ~(0xFF << 1);
	pi2c->pI2Cx->CR2 |= (pi2c->commConfig.slaveAdd << 1);

	// 3. Transfer Direction RD_WRN
	if(pi2c->commConfig.readNotWrite == 0)
	{
		// WRITE
		pi2c->state = I2C_STATE_IN_TX;
		pi2c->pI2Cx->CR2 &= ~(0x1 << 10); // Reset RD_WRN

		pi2c->pI2Cx->CR1 |= (1 << 1);  // Set TXIE
	}
	else
	{
		// READ
		pi2c->state = I2C_STATE_IN_RX;
		pi2c->pI2Cx->CR2 |= (0x1 << 10);	// Set RD_WRN

		pi2c->pI2Cx->CR1 |= (1 << 2);  // Set RXIE
	}

	// 4. HEAD10R Bit - Don't think I need to worry about this.

	// 5. # of bytes to be transferred NBYTES[7:0]
	if(pi2c->commConfig.buffLen > 255)
	{
		pi2c->pI2Cx->CR2 |= (0xFF << 16); // Set NBYTES to max value of 255
		pi2c->pI2Cx->CR2 |= (0x1 << 24);  // Set RELOAD bit
	}
	else
	{
		pi2c->pI2Cx->CR2 &= ~(0xFF << 16); 					   // Clear NBYTES field
		pi2c->pI2Cx->CR2 |= (pi2c->commConfig.buffLen << 16 ); // Set NBYTES
		pi2c->pI2Cx->CR2 &= ~(0x1 << 24);  					   // Reset RELOAD bit
	}


	// 6. Set AUTOEND bit
	pi2c->pI2Cx->CR2 |= (1 << 25);

	// 7. Enable I2C interrupts
	pi2c->pI2Cx->CR1 |= (1 << 4); // Set NACK interrupt enable
	pi2c->pI2Cx->CR1 |= (1 << 5); // Set STOP interrupt enable
	pi2c->pI2Cx->CR1 |= (1 << 6); // Set TCIE interrupt enable

	pi2c->pI2Cx->CR1 |= (1 << 7); // Set Error interrupts enable


	// 8. Set the start bit
	pi2c->pI2Cx->CR2 |= (1 << 13);

	return state;
}

void I2C_Slave_TXRX_INT(I2C_Handle_t *pi2c)
{
	pi2c->pI2Cx->CR1 &= ~(0xFE << 0); // Clear all interrupt enable bits

	pi2c->pI2Cx->CR1 |= (1 << 1);  // Set TXIE
	pi2c->pI2Cx->CR1 |= (1 << 2);  // Set RXIE
	pi2c->pI2Cx->CR1 |= (1 << 3);  // Set ADDRIE
	pi2c->pI2Cx->CR1 |= (1 << 5);  // Set STOP interrupt enable
	pi2c->pI2Cx->CR1 |= (1 << 7);  // Set Error interrupts enable
}


void I2C_INT_Reset(I2C_Handle_t *pi2c)
{
	pi2c->pI2Cx->ICR |= (0x3F << 8);  // clear all error flags
	pi2c->state = I2C_STATE_READY;
}

void I2C_EV_IRQHandler(I2C_Handle_t *pi2c)
{
	if(pi2c->pI2Cx->ISR & (1 << 3)) // if ADDR bit is set
	{
		if(pi2c->state == I2C_STATE_READY)
		{
			pi2c->state = I2C_STATE_SLAVE;
			if(pi2c->pI2Cx->ISR & (1 << 16)) // If Transmitting
			{
				pi2c->pI2Cx->ISR |= (1 << 0); // Set TXE
			}
		}
		pi2c->pI2Cx->ICR |= (1 << 3); // Clear ADDR flag
	}
	else if(pi2c->pI2Cx->ISR & (1 << 4)) // if NACKF set
	{
		pi2c->pI2Cx->ICR |= (1 << 4); // clear NACKF flag
		I2C_AppEvCallback(I2C_EV_M_NACK);
	}
	else if(pi2c->pI2Cx->ISR & (1 << 5)) // if STOPF flag set
	{
		pi2c->pI2Cx->ICR |= (1 << 5); // clear STOPF flag

		if(pi2c->state != I2C_STATE_SLAVE)
		{
			if(pi2c->commConfig.buffLen <= 0)
			{
				I2C_AppEvCallback(I2C_EV_TRANSACTION_SUCCESS);
			}
			else
			{
				I2C_AppEvCallback(I2C_EV_M_ERR);
			}
		}
		I2C_INT_Reset(pi2c);
	}
	else if(pi2c->pI2Cx->ISR & (1 << 1)) // TXIS flag set
	{
		if(pi2c->state != I2C_STATE_SLAVE)
		{
			*((volatile uint8_t *)(&pi2c->pI2Cx->TXDR)) = *pi2c->commConfig.pBuff;
			pi2c->commConfig.pBuff++;
			pi2c->commConfig.buffLen--;
		}
		else
		{
			*((volatile uint8_t *)(&pi2c->pI2Cx->TXDR)) = *((uint8_t *)I2C_SlaveTransmitCallback());
		}

	}
	else if(pi2c->pI2Cx->ISR & (1 << 2)) // RXNE flag set
	{
		if(pi2c->state != I2C_STATE_SLAVE)
		{
			*pi2c->commConfig.pBuff = *((volatile uint8_t *)(&pi2c->pI2Cx->RXDR));
			pi2c->commConfig.pBuff++;
			pi2c->commConfig.buffLen--;
		}
		else
		{
			I2C_SlaveReceiveCallback(*((volatile uint8_t *)(&pi2c->pI2Cx->RXDR)));
		}
	}
	else if(pi2c->pI2Cx->ISR & (1 << 7)) // TCR flag set!
	{
		if(pi2c->commConfig.buffLen > 255)
		{
			pi2c->pI2Cx->CR2 |= (0xFF << 16); // Set NBYTES to max value of 255
			pi2c->pI2Cx->CR2 |= (0x1 << 24);  // Set RELOAD bit
		}
		else
		{
			pi2c->pI2Cx->CR2 |= (pi2c->commConfig.buffLen << 16); // Set NBYTES
			pi2c->pI2Cx->CR2 &= ~(1 << 24); 	// Clear the RELOAD bit
		}
	}
}

void I2C_ER_IRQHandler(I2C_Handle_t *pi2c)
{
	I2C_INT_Reset(pi2c);
	I2C_AppEvCallback(I2C_EV_M_ERR);
}

void I2C_IRQ_Config(uint8_t irqNum, uint8_t enOrDi)
{
	// IRQ#31 - I2C1_EV
	// IRQ#32 - I2C1_ER
	uint8_t scRegNum = irqNum / NVIC_ISER_BIT_WIDTH;     // indicates ISER and ICER register #
	uint8_t scBitOffset = irqNum % NVIC_ISER_BIT_WIDTH;	 // indicates Bit Offset within the register

	if(enOrDi == ENABLE)
	{
		*(NVIC_ISER0_ADDR + scRegNum) |= (1 << scBitOffset); // set enable bit
	}
	else
	{
		*(NVIC_ICER0_ADDR + scRegNum) |= (1 << scBitOffset); // set enable-clear bit
	}

}
void I2C_IRQ_Priority(uint8_t irqNum, uint8_t pri)
{
	uint8_t priRegNum = irqNum / NVIC_PRI_FIELD_PER_PRI_REG;
	uint8_t priBitOffset = ((irqNum % NVIC_PRI_FIELD_PER_PRI_REG) * NVIC_PRI_FIELD_BIT_WIDTH) + (NVIC_PRI_FIELD_BIT_WIDTH - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + priRegNum) &= ~(0xF << priBitOffset);
	*(NVIC_IPR_BASEADDR + priRegNum) |= ( ((uint32_t) pri) << priBitOffset);
}

/*
 * I2C Application Callback WEAK implementations
 */
__attribute__((weak)) void I2C_SlaveReceiveCallback(uint8_t d)
{
	while(1);
}

__attribute__((weak)) uint8_t* I2C_SlaveTransmitCallback()
{
	while(1);
}

__attribute__((weak)) void I2C_AppEvCallback(uint8_t event)
{
	while(1);
}
