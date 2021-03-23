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
	uint32_t ChannelDiffInputMode; /* Up to 19 channels are represented with bits 0-18.
								    * Bits set to 1 indicate input channel is
								    * operating in differential mode. Otherwise single
								    * ended mode.
								    */
}ADC_Init_t;

typedef struct{
	ADC_RegDef_t       *pADCx;
	ADC_Init_t			Init;
}ADC_Handle_t;




/*
 * Configures ADC clock source to be SYSCLK for all ADCs
 *
 * Note: Although not supported here, ADC can also be clocked from:
 * 1. PLLADC1CLK, PLLADC2CLK
 * 2. HCLK
 *
 */
void ADC_EnableClockFromSYSCLK();

/*
 * Configures initial settings and enables the specific ADC passed in.
 *
 *
 * @param pADC: pointer to ADC_Handle_t structure
 */
void ADCx_Init(ADC_Handle_t *pADC);


/*
 * Disables the specific ADC passed into the handle
 *
 * @param pADC: pointer to ADC_Handle_t structure
 */
void ADCx_Disable(ADC_Handle_t *pADC);

/*
 * Powers down all ADCs. The ADC voltage regulator is turned off.
 * The calibration values will need to be recalibrated.
 *
 * @param pADC: pointer to ADC_Handle_t structure
 */
void ADC_PowerDown(ADC_Handle_t *pADC);


/*
 * TODO:
 * ADC calibration value may be configured manually to avoid waiting for
 * recalibration
 */
void ADCx_SetCalibrationValue(ADC_Handle_t *pADC);
void ADCx_GetCalibrationValue(ADC_Handle_t *pADC);











// ------------------------- CONTROL MACROS ------------------------------
#define ADC_VREG_SETUP_DELAY_US (20UL)






// ---------------- REGISTER MACROS (EACH ADC, NOT SHARED) ---------------
// ISR - Interrupt and Status Register
#define ISR_ADRDY_BIT_POS      (0)

// CR - Control Register
#define CR_ADCAL_BIT_POS       (31)
#define CR_ADCALDIF_BIT_POS    (30)
#define CR_DEEPPWD_BIT_POS     (29)
#define CR_ADVREGEN_BIT_POS    (28)
#define CR_JADSTP_BIT_POS	    (5)
#define CR_ADSTP_BIT_POS	    (4)
#define CR_JADSTART_BIT_POS     (3)
#define CR_ADSTART_BIT_POS      (2)
#define CR_ADDIS_BIT_POS        (1)
#define CR_ADEN_BIT_POS		    (0)

// DIFSEL - Differential Mode Selection Register
#define DIFSEL_LOWER_BITS_USED_COUNT (19)


// --------------------- COMMON REGISTER MACROS --------------------------
// CCR - Common Control Register
#define CCR_CKMODE_BIT_POS  (16)


#endif /* INC_ADC_DRIVER_H_ */
