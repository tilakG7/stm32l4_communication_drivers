/*
 * stm32l47x_gpio_driver.h
 *
 *  Created on: Aug 31, 2020
 *      Author: Tilak Gupta
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32l4xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx; 			 // Base address of the port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // GPIO pin configuration settings

}GPIO_Handle_t;


/*
 * GPIO Pin Numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*
 * GPIO Pin Possible Modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFCN	2
#define GPIO_MODE_ANALOG	3

#define GPIO_MODE_IT_FT		4  // falling edge triggers interrupt
#define GPIO_MODE_IT_RT		5  // rising edge triggers interrupt
#define GPIO_MODE_IT_RFT	6  // rising or falling edge triggers interrupt


/*
 * GPIO Speed Possibilities
 */
#define GPIO_LOW_SPEED		0
#define GPIO_MED_SPEED		1
#define GPIO_HIGH_SPEED		2
#define GPIO_VHIGH_SPEED	3


/*
 * GPIO Pull-up/pull-down possibilities
 */
#define GPIO_PIN_NO_PU_PD	0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/*
 * GPIO Output Type Possibilities
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1


/*
 * GPIO Alternate Function Select
 */
#define GPIO_ALT_FCN_0		0
#define GPIO_ALT_FCN_1		1
#define GPIO_ALT_FCN_2		2
#define GPIO_ALT_FCN_3		3
#define GPIO_ALT_FCN_4		4
#define GPIO_ALT_FCN_5		5
#define GPIO_ALT_FCN_6		6
#define GPIO_ALT_FCN_7		7
#define GPIO_ALT_FCN_8		8
#define GPIO_ALT_FCN_9		9
#define GPIO_ALT_FCN_10		10
#define GPIO_ALT_FCN_11		11
#define GPIO_ALT_FCN_12		12
#define GPIO_ALT_FCN_13		13
#define GPIO_ALT_FCN_14		14
#define GPIO_ALT_FCN_15		15


/*
 * GPIO SYSCFG Port Value
 */
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0:\
										(x == GPIOB) ? 1:\
										(x == GPIOC) ? 2:\
										(x == GPIOD) ? 3:\
										(x == GPIOE) ? 4:\
										(x == GPIOF) ? 5:\
										(x == GPIOG) ? 6:\
										(x == GPIOH) ? 7:7)



/********************************************************************************************
 * 								APIs Supported By This Driver
 ********************************************************************************************/


/*
 * GPIO Clock Control
 * Enables the passed in GPIO ports clock.
 *
 */
void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


/*
 * GPIO Initi  - configures GPIO settings for certain pins
 * GPIO Deinit - resets the GPIO port settings
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * GPIO Read and Write
 */
uint8_t  GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_WriteTogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);



/*
 * IRQ configuration and handling
 */
void GPIO_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_PriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);









#endif /* INC_GPIO_DRIVER_H_ */
