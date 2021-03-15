/*
 * adc_driver.c
 *
 *  Created on: Mar 14, 2021
 *      Author: tilak
 */

#include "adc_driver.h"

void ADC_ConfigClock(ADC_Handle_t *pADC);
{
	pADC->pCommonADCx->CCR &= ~CCR_CKMODE_MASK;

}
