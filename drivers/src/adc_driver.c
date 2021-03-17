/*
 * adc_driver.c
 *
 *  Created on: Mar 14, 2021
 *      Author: Tilak Gupta
 */

#include "adc_driver.h"


void ADC_EnableClockFromSYSCLK()
{
	RCC->CCIPR |= (0x3 << 28); // select SYSCLK instead of PLLADC1CLK, PLLADC2CLK
	ADC_COMMON_BASEADDR->CCR &= (0x3 << CCR_CKMODE_BIT_POS); // CKMODE=00 selects CK_ADCx (SYSCLK)
															 // instead of HCLK
	ADC_PCLK_EN();             // enable clock delivery to peripheral
}

void ADCx_Init(ADC_Handle_t *pADC)
{
	pADC->CR &= ~(0x1 << CR_DEEPPWD_BIT_POS); // exit from deep power down mode

	if( !(pADC->CR & (0x1 << CR_ADVREGEN_BIT_POS)) )
	{
		pADC->CR |= (0x1 << CR_ADVREGEN_BIT_POS); // enable internal voltage regulator

		// 20us delay before ADC voltage regulator to power up
		waitLopCount =  (SystemCoreClock / 10000000UL) * ADC_VREG_SETUP_DELAY_US; // (cycles/us) * (20us)
		while (waitLopCount--) {
				asm volatile ("");
		}
	}

	// Program the input mode of ADC's channels
	pADC->DIFSEL |= pADC->Init.ChannelDiffInputMode \
			& ((0x1UL << DIFSEL_LOWER_BITS_USED_COUNT) - 1); // register uses only 19 least significant bits

	// Perform ADC Calibration
	// Single ended input mode and differential input mode each require
	// their own calibrations if used. Both are performed here.
	pADC->CR &= ~(0x1UL << CR_EN_BIT_POS); // Ensure ADC is disabled
	pADC->CR |= (0x1UL << CR_ADCALDIF_BIT_POS); // set differential mode
	pADC->CR |= (0x1UL << CR_ADCAL_BIT_POS); // enable ADC calibration
	while(pADC->CR & (0x1UL << CR_ADCAL_BIT_POS)); // block until calibration complete

	pADC->CR &= ~(0x1UL << CR_ADCALDIF_BIT_POS); // set single-ended mode
	pADC->CR |= (0x1UL << CR_ADCAL_BIT_POS); // enable ADC calibration
	while(pADC->CR & (0x1UL << CR_ADCAL_BIT_POS)); // block until calibration complete

	//Configure channels, launch channels
	//Regular conversion by setting ADSTART = 1

	// Enable ADC - wait 4 cycles after Adcal cleared by HW
	pADC->ISR |= (0x1UL << ISR_ADRDY_BIT_POS);
	pADC->CR |= (0x1UL << CR_ADEN_BIT_POS);
	while(!(pADC->ISR & (0x1UL << ISR_ADRDY_BIT_POS))); // wait for ADC stabilization time

	//ADRDY IE interrupt


}

void ADCx_Disable(ADC_Handle_t *pADC)
{
	// ensure ADSTART & JADSTART are 0
	if(READ_BIT(pADC->pADCx->CR, CR_ADSTART_BIT_POS))
	{
		SET_BIT(pADC->pADCx->CR, CR_ADSTP_BIT_POS);
		while(READ_BIT(pADC->pADCx->CR, CR_ADSTART_BIT_POS)); // wait for conversion to stop
	}
	if(READ_BIT(pADC->pADCx->CR, CR_JADSTART_BIT_POS))
	{
		SET_BIT(pADC->pADCx->CR, CR_JADSTP_BIT_POS);
		while(READ_BIT(pADC->pADCx->CR, CR_JADSTART_BIT_POS)); // wait for conversion to stop
	}

	SET_BIT(pADC->CR, CR_ADDIS_BIT_POS);
}

void ADC_PowerDown()
{
	pADC->CR |= (0x1 << CR_DEEPPWD_BIT_POS); // go to deep power down mode
											     // voltage regulator automatically disabled
		 	 	 	 	 	 	 	 	 	 	 // calibration data lost
}
