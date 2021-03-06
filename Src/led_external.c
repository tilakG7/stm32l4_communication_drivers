/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


#include "stm32l4xx.h"
#include "stm32l47x_gpio_driver.h"
#include <stdint.h>

int main(void)
{
	GPIO_Handle_t ledHandle = {{GPIOC}, {GPIO_PIN_NO_1, GPIO_MODE_OUT, GPIO_VHIGH_SPEED, GPIO_PIN_NO_PU_PD, GPIO_OP_TYPE_PP}};
	GPIO_Handle_t buttonHandle = {{GPIOC}, {GPIO_PIN_NO_0, GPIO_MODE_IN, GPIO_VHIGH_SPEED, GPIO_PIN_PU}};
	GPIO_ClockControl(GPIOC, ENABLE);

	GPIO_Init(&ledHandle);
	GPIO_Init(&buttonHandle);


    /* Loop forever */
	while(1)
	{

		if(GPIO_ReadPin(GPIOC, GPIO_PIN_NO_0) == RESET)
		{
			for(uint32_t i=0; i < 100000; i++);
			GPIO_WriteTogglePin(GPIOC, ledHandle.GPIO_PinConfig.GPIO_PinNumber);
		}
//		else
//		{
//			GPIO_WritePin(GPIOC, ledHandle.GPIO_PinConfig.GPIO_PinNumber, SET);
//		}
//		GPIO_WriteTogglePin(GPIOB, led2Handle.GPIO_PinConfig.GPIO_PinNumber);
//		GPIO_WriteTogglePin(GPIOA, led1Handle.GPIO_PinConfig.GPIO_PinNumber);
	}

}


