/*
 * stm32l4x.c
 * Function to enable/disable interrupt in the ARM Cortex-M4 NVIC Peripheral
 * Function to set priority of specific interrupt the ARM Cortex-M4 NVIC Peripheral
 *  Created on: Mar 13, 2021
 *      Author: tilak
 */

#include "stm32l4xx.h"

void Arm_IRQConfig(uint8_t IRQNum, uint8_t EnorDi)
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

void Arm_PriorityConfig(uint8_t IRQNum, uint32_t IRQPriority)
{
	uint8_t iprNum = IRQNum / 4;
	uint8_t iprPos = IRQNum % 4;

	(*(NVIC_IPR_BASEADDR + iprNum)) = ( (*(NVIC_IPR_BASEADDR + iprNum)) & ~(0xFF << (iprPos*8)) ) | ( IRQPriority << ((8 - NO_PR_BITS_IMPLEMENTED) + (iprPos*8)) );
}
