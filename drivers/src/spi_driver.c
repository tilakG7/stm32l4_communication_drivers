/*
 * stm32l47x_spi_driver.c
 *
 *  Created on: Sep 4, 2020
 *      Author: tilak
 */


#include <spi_driver.h>

void SPI_SetPCLK(SPI_Handle_t *pSpiHandle, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		if(pSpiHandle->pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSpiHandle->pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSpiHandle->pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSpiHandle->pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSpiHandle->pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSpiHandle->pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

void SPI_Init(SPI_Handle_t *pSpiHandle)
{


	// 1. Device Mode - Slave or Master
	if(pSpiHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER)
	{
		pSpiHandle->pSPIx->CR1 |= (0x1 << 2);
	}
	else if(pSpiHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_SLAVE)
	{
		pSpiHandle->pSPIx->CR1 &= ~(0x1 << 2);
	}

	// 2. Bus Config - FULL_DUPLEX, HALF_DUPLEX, SimplexRX/TX
	if(pSpiHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX)
	{
		pSpiHandle->pSPIx->CR1 &= ~(0x1 << 15);
		pSpiHandle->pSPIx->CR1 &= ~(0x1 << 10); // ensure RXONLY is disabled
	}
	else if(pSpiHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX)
	{
		pSpiHandle->pSPIx->CR1 |= (0x1 << 15);
		//TODO: would need to configure whether transmitting or receiving
	}
	else if(pSpiHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
	{
		pSpiHandle->pSPIx->CR1 &= ~(0x1 << 15);	// ensure 2 line mode is
		pSpiHandle->pSPIx->CR1 |=  (0x1 << 10);
	}

	// 3. Configure Pre-scaler
	pSpiHandle->pSPIx->CR1 &= ~(0x7 << 3);
	pSpiHandle->pSPIx->CR1 |= (pSpiHandle->SPI_Config.SPI_ClkSpeed << 3);

	// 4. Configure CPHA
	pSpiHandle->pSPIx->CR1 &= ~(0x1 << 0);
	pSpiHandle->pSPIx->CR1 |= (pSpiHandle->SPI_Config.SPI_CPHA << 0);

	// 5. Configure CPOL
	pSpiHandle->pSPIx->CR1 &= ~(0x1 << 1);
	pSpiHandle->pSPIx->CR1 |= (pSpiHandle->SPI_Config.SPI_CPOL << 1);

	//6. DFF
	pSpiHandle->pSPIx->CR2 &= ~(0xF << 8);
	if(pSpiHandle->SPI_Config.SPI_DFF == SPI_DFF_8)
	{
		pSpiHandle->pSPIx->CR2 |= (0x7 << 8);
	}
	else if(pSpiHandle->SPI_Config.SPI_DFF == SPI_DFF_16)
	{
		pSpiHandle->pSPIx->CR2 |= (0xF << 8);
	}

	//7. Slave SW Management ON/OFF
	pSpiHandle->pSPIx->CR1 &= ~(0x1 << 9);
	pSpiHandle->pSPIx->CR1 |= (pSpiHandle->SPI_Config.SPI_SSM << 9);

	// If Slave SW Mgmt enabled, configure SSI high
	if(pSpiHandle->SPI_Config.SPI_SSM == ENABLE)
	{
		pSpiHandle->pSPIx->CR1 |= (0x1 << 8);
	}
	else
	{
		// If Slave SW Mgmt disabled, configure the SSOE bit.
		pSpiHandle->pSPIx->CR2 |= (0x1 << 2);
	}
}

void SPI_EnDi(SPI_Handle_t *pSpiHandle, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		pSpiHandle->pSPIx->CR1 |= (0x1 << 6);
	}
	else
	{
		pSpiHandle->pSPIx->CR1 &= ~(0x1 << 6);
	}
}
void SPI_Deinit(SPI_Handle_t *pSpiHandle)
{
	if(pSpiHandle->pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSpiHandle->pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSpiHandle->pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/*
 * SPI Communication - Blocking
 */
void SPI_RX(SPI_Handle_t *pSpiHandle, uint8_t *pRxBuff, uint32_t buffLen)
{
	while(buffLen > 0)
	{
		//1. Wait until RXNE is set
		while(pSpiHandle->pSPIx->SR & (0x1));

		//2. Read DR
		if(pSpiHandle->SPI_Config.SPI_DFF == SPI_DFF_16)
		{
			*pRxBuff = *((__vo uint16_t *)(&pSpiHandle->pSPIx->DR));
			buffLen -=2;
			pRxBuff++;
			pRxBuff++;
		}
		else
		{
			*pRxBuff = *((__vo uint8_t *)(&pSpiHandle->pSPIx->DR));
			buffLen--;
			pRxBuff++;
		}
	}
}

void SPI_TX(SPI_Handle_t *pSpiHandle, uint8_t *pTxBuff, uint32_t buffLen)
{
	while(buffLen > 0)
	{
		// 1. wait TX empty status bit is set
		while( SPI_GetStatus(pSpiHandle->pSPIx, SPI_FLAG_TXE) != SET );

		// 2. Check DFF and send data
		if(pSpiHandle->SPI_Config.SPI_DFF == SPI_DFF_16)
		{
			pSpiHandle->pSPIx->DR = *((uint16_t *)pTxBuff);
			buffLen -= 2;
			pTxBuff += 2;
		}
		else
		{
			(*(__vo uint8_t*)(&pSpiHandle->pSPIx->DR)) = *pTxBuff;
			buffLen--;
			pTxBuff++;
		}
	}

}

uint8_t SPI_RX_IT(SPI_Handle_t *pSpiHandle, uint8_t *pRxBuff, uint32_t buffLen)
{
	uint8_t state = pSpiHandle->rxState;

	if(state != SPI_STATE_IN_RX)
	{
		// 1. Save rx buffer address and length in handle variables
		pSpiHandle->pRX = pRxBuff;
		pSpiHandle->rxLen = buffLen;

		// 2. Set rx state as busy in rx
		pSpiHandle->rxState = SPI_STATE_IN_RX;

		// 3. Set RXNEIE RX Buffer not empty interrupt enable
		pSpiHandle->pSPIx->CR2 |= (0x1 << 6);
	}

	return state;
}

uint8_t SPI_TX_IT(SPI_Handle_t *pSpiHandle, uint8_t *pTxBuff, uint32_t buffLen)
{
	uint8_t state = pSpiHandle->txState;

	if(state != SPI_STATE_IN_TX)
	{
		// 1. Save tx buffer address and length in handle variables
		pSpiHandle->pTX = pTxBuff;
		pSpiHandle->txLen = buffLen;

		// 2. Set tx state as busy in tx
		pSpiHandle->txState = SPI_STATE_IN_TX;

		// 3. Set TXEIE:Txbufferemptyinterruptenable
		pSpiHandle->pSPIx->CR2 |= (0x1 << 7);
	}

	return state;
}

/*
 * SPI Interrupt Handling
 */
void SPI_SetInterrupt(uint8_t irqNum, uint8_t enOrDi)
{
	int serNum = irqNum / NVIC_ISER_BIT_WIDTH; // set-enable register numbers range from 0 - 7. Each is 32 bits wide
	int setBitShift = irqNum % NVIC_ISER_BIT_WIDTH; // each set-enable register is 32 bits wide

	if(enOrDi == ENABLE)
	{
		// write to set register
		*(NVIC_ISER0_ADDR + serNum) |= (0x1 << setBitShift);
	}
	else
	{
		// write to clear register
		*(NVIC_ICER0_ADDR + serNum) |= (0x1 << setBitShift);
	}

}

void SPI_SetIRQPriority(uint8_t irqNum, uint8_t pri)
{
	int priRegNum = irqNum / (NVIC_PRI_FIELD_BIT_WIDTH * 4);
	int priFieldBitShift = (irqNum % NVIC_PRI_FIELD_PER_PRI_REG) * NVIC_PRI_FIELD_BIT_WIDTH;

	*(NVIC_IPR_BASEADDR + priRegNum) &= ~(0xF << priFieldBitShift);
	*(NVIC_IPR_BASEADDR + priRegNum) |= ( ((uint32_t)pri) << (priFieldBitShift + (8 - NO_PR_BITS_IMPLEMENTED)) );
}

void SPI_HandleIRQ(SPI_Handle_t *pSpiHandle)
{
	// Check what might have caused error

	// Check RXNE and RXNEIE
	if((pSpiHandle->pSPIx->SR & (1 << 0)) && (pSpiHandle->pSPIx->CR2 & (1 << 6)))
	{
		SPI_Handle_RXNE(pSpiHandle);
	}
	// Check TXE and TXEIE
	else if((pSpiHandle->pSPIx->SR & (1 << 1)) && (pSpiHandle->pSPIx->CR2 & (1 << 7)))
	{
		SPI_Handle_TXE(pSpiHandle);
	}
	// add code to check MODF, OVR, FRE, CRCERR handling errors
}

/*
 * SPI Helper Functions
 */

uint8_t SPI_GetStatus(SPI_RegDef_t *pSpiRegDef, SPI_StatusFlag_t flag)
{
	if(pSpiRegDef->SR & (0x1 << ((uint32_t)flag)) )
	{
		return SET;
	}
	return RESET;
}

static void SPI_Handle_RXNE(SPI_Handle_t *pSpiHandle)
{
	// Read based on DFF
	if(pSpiHandle->SPI_Config.SPI_DFF == SPI_DFF_16)
	{
		*pSpiHandle->pRX = *((__vo uint16_t *)(&pSpiHandle->pSPIx->DR));
		pSpiHandle->rxLen -=2;
		pSpiHandle->pRX++;
		pSpiHandle->pRX++;
	}
	else
	{
		*pSpiHandle->pRX = *((__vo uint8_t *)(&pSpiHandle->pSPIx->DR));
		pSpiHandle->rxLen--;
		pSpiHandle->pRX++;
	}

	if(pSpiHandle->rxLen <= 0)
	{
		// 1. reset rxBuff and rxLen
		pSpiHandle->pRX = 0;
		pSpiHandle->rxLen = 0;

		// 2. Clear RX SPI state
		pSpiHandle->rxState = SPI_STATE_READY;

		// 3. Clear RXNEIE RX Buffer not empty interrupt enable
		pSpiHandle->pSPIx->CR2 &= ~(0x1 << 6);

		SPI_ApplicationEventCallback(pSpiHandle, SPI_EVENT_RX_COMPLETE); // application must implement this
	}
}
static void SPI_Handle_TXE(SPI_Handle_t *pSpiHandle)
{
	if(pSpiHandle->SPI_Config.SPI_DFF == SPI_DFF_16)
	{
		pSpiHandle->pSPIx->DR = *((uint16_t *)pSpiHandle->pTX);
		pSpiHandle->txLen -= 2;
		pSpiHandle->pTX += 2;
	}
	else
	{
		(*(__vo uint8_t*)(&pSpiHandle->pSPIx->DR)) = *pSpiHandle->pTX;
		pSpiHandle->txLen--;
		pSpiHandle->pTX++;
	}

	if(pSpiHandle->txLen <= 0)
	{
		// reset txeie bit
		pSpiHandle->pSPIx->CR2 &= ~(0x1 << 7);
		// reset other parameters of handle
		pSpiHandle->pTX = 0;
		pSpiHandle->txState = SPI_STATE_READY;
		pSpiHandle->txLen = 0;
		SPI_ApplicationEventCallback(pSpiHandle, SPI_EVENT_TX_COMPLETE); // application must implement this
	}
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSpiHandle, uint8_t event)
{
	// needs to be implemented by the application!
}
