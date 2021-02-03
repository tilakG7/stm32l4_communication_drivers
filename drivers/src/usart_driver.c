/*
 * usart_driver.c
 *
 *  Created on: Sep. 16, 2020
 *      Author: tilak
 */

#include "usart_driver.h"

void USART_PCLKControl(USART_Handle_t *p, uint8_t enOrDi)
{
	if(enOrDi == ENABLE)
	{
		if(p->pReg == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(p->pReg == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(p->pReg == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(p->pReg == UART4)
		{
			UART4_PCLK_EN();
		}
		else
		{
			UART5_PCLK_EN();
		}
	}
	else
	{
		if(p->pReg == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(p->pReg == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(p->pReg == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(p->pReg == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(p->pReg == UART5)
		{
			UART5_PCLK_DI();
		}
	}
}
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// 0. Disable USART
	pUSARTHandle->pReg->CR1 &= ~(1 << 0);

	// 1. USART Mode: TXRX
	if(pUSARTHandle->c.USART_Mode == USART_MODE_ONLY_TX)
	{
		pUSARTHandle->pReg->CR1 &= ~(1 << 2);
		pUSARTHandle->pReg->CR1 |= (1 << 3);
	}
	else if(pUSARTHandle->c.USART_Mode == USART_MODE_ONLY_RX)
	{
		pUSARTHandle->pReg->CR1 &= ~(1 << 3);
		pUSARTHandle->pReg->CR1 |= (1 << 2);
	}
	else
	{
		pUSARTHandle->pReg->CR1 |= (1 << 2);
		pUSARTHandle->pReg->CR1 |= (1 << 3);
	}

	// 2. Baud Rate Generation
	pUSARTHandle->pReg->CR1 |= (1 << 15); // set oversampling by 8 true
	uint16_t USARTDIV = (4000000 * 2) / pUSARTHandle->c.USART_Baud; // assuming 4Mhz internal clock
	pUSARTHandle->pReg->BRR = (USARTDIV & 0xFFF0) | ((USARTDIV & 0xF) >> 1);

	// 3. Parity Control
	if(pUSARTHandle->c.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		pUSARTHandle->pReg->CR1 |= (0x3 << 9); // set odd parity (Bit9) and parity control enable (Bit10)
	}
	else if(pUSARTHandle->c.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		pUSARTHandle->pReg->CR1 |= (1 << 10); // set parity control enable bit
		pUSARTHandle->pReg->CR1 &= ~(1 << 9); // clear PS bit to set even parity
	}
	else if(pUSARTHandle->c.USART_ParityControl == USART_PARITY_DISABLE)
	{
		pUSARTHandle->pReg->CR1 &= ~(1 << 10); // clear parity control enable bit
	}

	// 4. Data word length
	if(pUSARTHandle->c.USART_DataWordLength == USART_WORDLEN_7BITS)
	{
		// Clear M0, set M1
		pUSARTHandle->pReg->CR1 |= (1 << 28); // set M1
		pUSARTHandle->pReg->CR1 &= ~(1 << 12); // clear M0
	}
	else if(pUSARTHandle->c.USART_DataWordLength == USART_WORDLEN_8BITS)
	{
		// Clear M0 & M1
		pUSARTHandle->pReg->CR1 &= ~(1 << 28); // clear M1
		pUSARTHandle->pReg->CR1 &= ~(1 << 12); // clear M0
	}
	else if(pUSARTHandle->c.USART_DataWordLength == USART_WORDLEN_9BITS)
	{
		// Clear M1, set M0
		pUSARTHandle->pReg->CR1 &= ~(1 << 28); // clear M1
		pUSARTHandle->pReg->CR1 |= (1 << 12); // set M0
	}

	// 5. Number of stop bits
	pUSARTHandle->pReg->CR2 &= ~(0x3 << 12); // Clear Stop Bits Field
	pUSARTHandle->pReg->CR2 |= (pUSARTHandle->c.USART_NoStopBits << 12);

	// 6. HW Flow Control
	pUSARTHandle->pReg->CR3 &= ~(0x3 << 8); // Clear CTSE and RTSE
	pUSARTHandle->pReg->CR3 |= (pUSARTHandle->c.USART_HWFlowControl << 8);

	// 7. Enable UART
	pUSARTHandle->pReg->CR1 |= (1 << 0);

}
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}
	else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}
}


/*
 * Data Send and Receive
 */

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pReg, USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->c.USART_DataWordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pReg->TDR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->c.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pReg->TDR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pReg,USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pReg, USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->c.USART_DataWordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->c.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pReg->RDR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer += 2;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pReg->RDR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->c.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = *((uint8_t *)pUSARTHandle->pReg->RDR);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = ((uint8_t)0x7F) & (*((uint8_t *)pUSARTHandle->pReg->RDR));

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->txState;

	if(txstate != USART_STATE_TX)
	{
		pUSARTHandle->txBufferLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->txState = USART_STATE_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pReg->CR1 |= ( 1 << 7 ); // set TXEIE


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pReg->CR1 |= (1 << 6); // set TCIE
	}

	return txstate;
}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->rxState;

	if(rxstate != USART_STATE_RX)
	{
		pUSARTHandle->rxBufferLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->rxState = USART_STATE_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pReg->CR1 |= (1 << 5);

	}

	return rxstate;

}




/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	uint8_t scRegNum = IRQNumber / NVIC_ISER_BIT_WIDTH; // set/clear register number
	uint8_t scBitOffset = IRQNumber % NVIC_ISER_BIT_WIDTH; // bit offset in set/clear register pertaining to IRQ#

	if(EnorDi == ENABLE)
	{
		*(NVIC_ISER0_ADDR + scRegNum) |= (1 << scBitOffset); // write to enable register
	}
	else
	{
		*(NVIC_ICER0_ADDR + scRegNum) |= (1 << scBitOffset); // write to clear register
	}
}
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t priRegNum = IRQNumber / (NVIC_PRI_FIELD_PER_PRI_REG);
	uint8_t priBitOffset = ((IRQNumber % NVIC_PRI_FIELD_PER_PRI_REG) * NVIC_PRI_FIELD_BIT_WIDTH) + (NVIC_PRI_FIELD_BIT_WIDTH - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + priRegNum) |= (IRQPriority << priBitOffset);
}
void USART_IRQHandling(USART_Handle_t *pHandle)
{
	if(pHandle->txState == USART_STATE_TX)
	{
		if(USART_GetFlagStatus(pHandle->pReg, USART_FLAG_TC) && pHandle->txBufferLen <= 0)
		{
			pHandle->txState = USART_STATE_READY;
			pHandle->pTxBuffer = 0;
			pHandle->txBufferLen = 0;
		}
		else if(USART_GetFlagStatus(pHandle->pReg, USART_FLAG_TXE))
		{
			if(pHandle->txBufferLen > 0)
			{
				 //Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pHandle->c.USART_DataWordLength == USART_WORDLEN_9BITS)
				{
					uint16_t *pdata;
					//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pHandle->pTxBuffer;
					pHandle->pReg->TDR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pHandle->c.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer. so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pHandle->pTxBuffer++;
						pHandle->pTxBuffer++;
						pHandle->txBufferLen--;
						pHandle->txBufferLen--;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pHandle->pTxBuffer++;
						pHandle->txBufferLen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pHandle->pReg->TDR = (*pHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pHandle->pTxBuffer++;
					pHandle->txBufferLen--;
				}
			}
		}
	}

	if(pHandle->rxState == USART_STATE_RX)
	{
		if(USART_GetFlagStatus(pHandle->pReg, USART_FLAG_RXNE) && pHandle->rxBufferLen > 0)
		{
			//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
			if(pHandle->c.USART_DataWordLength == USART_WORDLEN_9BITS)
			{
				//We are going to receive 9bit data in a frame

				//check are we using USART_ParityControl control or not
				if(pHandle->c.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used. so, all 9bits will be of user data

					//read only first 9 bits. so, mask the DR with 0x01FF
					*((uint16_t*) pHandle->pRxBuffer) = (pHandle->pReg->RDR  & (uint16_t)0x01FF);

					//Now increment the pRxBuffer two times
					pHandle->pRxBuffer += 2;
				}
				else
				{
					//Parity is used, so, 8bits will be of user data and 1 bit is parity
					 *pHandle->pRxBuffer = (pHandle->pReg->RDR  & (uint8_t)0xFF);

					 //Increment the pRxBuffer
					 pHandle->pRxBuffer++;
				}
			}
			else
			{
				//We are going to receive 8bit data in a frame

				//check are we using USART_ParityControl control or not
				if(pHandle->c.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used , so all 8bits will be of user data

					//read 8 bits from DR
					 *pHandle->pRxBuffer = *((uint8_t *)pHandle->pReg->RDR);
				}

				else
				{
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity

					//read only 7 bits , hence mask the DR with 0X7F
					 *pHandle->pRxBuffer = ((uint8_t)0x7F) & (*((uint8_t *)pHandle->pReg->RDR));

				}

				//increment the pRxBuffer
				pHandle->pRxBuffer++;
			}
		}
		if(pHandle->rxBufferLen <= 0)
		{
			pHandle->rxState = USART_STATE_READY;
			pHandle->rxBufferLen = 0;
			pHandle->pRxBuffer = 0;
		}
	}
}

/*
 * Other Peripheral Control APIs
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	return (pUSARTx->ISR & (1 << FlagName));
}
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	if(StatusFlagName == USART_FLAG_TCGBT)
	{
		pUSARTx->ICR |= (1 << 7);
	}
	else
	{
		pUSARTx->ICR |= ( 1 << StatusFlagName);
	}
}

/*
 * Application callback
 */
__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{
	while(1);
}
