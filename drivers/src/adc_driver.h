/*
 * adc_driver.h
 *
 *  Created on: Mar 14, 2021
 *      Author: tilak
 */

#ifndef INC_ADC_DRIVER_H_
#define INC_ADC_DRIVER_H_

#include "stm32l4xx.h"


typedef struct{
	ADC_RegDef_t       *pADCx;
	ADC_CommonRegDef_t *pCommonADCx;

}ADC_Handle_t;


/*
 * Configures clock source to be SYSCLK
 * ADC can alternatively be run from PLLSA1, PLLSA2 or AHB
 */
void ADC_ConfigClock(ADC_Handle_t *pADC);





// CCR Register Masks
#define CCR_CKMODE_MASK (0x3 << 16)

#endif /* INC_ADC_DRIVER_H_ */
