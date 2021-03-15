/*
 * adc_driver.c
 *
 *  Created on: Mar 14, 2021
 *      Author: tilak
 */

#include "adc_driver.h"


void ADC_EnableClockFromSYSCLK(ADC_Handle_t *pADC)
{
	RCC->CCIPR |= (0x3 << 28); // select SYSCLK instead of PLLADC1CLK, PLLADC2CLK
	ADC_COMMON_BASEADDR->CCR &= (0x3 << CCR_CKMODE_BIT_POS); // CKMODE=00 selects CK_ADCx (SYSCLK)
															 // instead of HCLK
	ADC_PCLK_EN();             // enable clock delivery to peripheral
}

void ADC_Init(ADC_Handle_t *pADC)
{
	pADC->CR &= ~(0x1 << CR_DEEPPWD_BIT_POS); // exit from deep power down mode

	if( !(pADC->CR & (0x1 << CR_ADVREGEN_BIT_POS)) )
	{
		pADC->CR |= (0x1 << CR_ADVREGEN_BIT_POS); // enable internal voltage regulator
		// TODO: add 20us delay before ADC voltage reg to power up
	}
}
