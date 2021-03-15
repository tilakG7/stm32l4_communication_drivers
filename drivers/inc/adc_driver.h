/*
 * adc_driver.h
 *
 *  Macros and structures for the Analog to Digital Converter (ADC)
 *
 *  Created on:  Mar 14, 2021
 *  Author:      Tilak Gupta
 */

#ifndef INC_ADC_DRIVER_H_
#define INC_ADC_DRIVER_H_

#include "stm32l4xx.h"


typedef struct{
	ADC_RegDef_t       *pADCx;

}ADC_Handle_t;


/*
 * Configures ADC clock source to be SYSCLK
 *
 * Note: Although not supported here, ADC can also be clocked from:
 * 1. PLLADC1CLK, PLLADC2CLK
 * 2. HCLK
 *
 * @param pADC: pointer to ADC_Hande_t structure
 */
void ADC_EnableClockFromSYSCLK(ADC_Handle_t *pADC);

/*
 * Configures initial settings and enables ADC
 *
 * @param pADC: pointer to ADC_Handle_t structure
 */
void ADC_Init(ADC_Handle_t *pADC);




// ---------------- REGISTER MACROS (EACH ADC, NOT SHARED) ---------------
// CR - Control Register
#define CR_DEEPPWD_BIT_POS     (29)
#define CR_ADVREGEN_BIT_POS    (28)


// --------------------- COMMON REGISTER MACROS --------------------------
// CCR - Common Control Register
#define CCR_CKMODE_BIT_POS  (16)


#endif /* INC_ADC_DRIVER_H_ */
