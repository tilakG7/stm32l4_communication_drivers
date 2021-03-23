/*
 * 011_USART.c
 *
 *  Created on: Sep. 16, 2020
 *      Author: Tilak Gupta
 *
 *  Send a lower case string over UART and receive the uppercased version.
 *
 *  This code sends data over UART to another application running on the Arduino.
 *  The code then receives data over UART, which has now been capitalized.
 */

#include <gpio_driver.h>
#include "usart_driver.h"



void gpio_init()
{
	GPIO_Handle_t h;

	h.pGPIOx = GPIOA;

	h.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFCN;
	h.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VHIGH_SPEED;
	h.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	h.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; 	// open drain so other device can also comm.
	h.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALT_FCN_8;

	h.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;

	GPIO_ClockControl(GPIOA, ENABLE);
	GPIO_Init(&h);

	h.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&h);
}

USART_Handle_t uh;
void usart_int_init()
{
	uh.c.USART_Mode = USART_MODE_TXRX;
	uh.c.USART_Baud = USART_STD_BAUD_115200;
	uh.c.USART_NoStopBits = USART_STOPBITS_1;
	uh.c.USART_DataWordLength = USART_WORDLEN_8BITS;
	uh.c.USART_ParityControl = USART_PARITY_DISABLE;
	uh.c.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	uh.rxState = USART_STATE_READY;
	uh.txState = USART_STATE_READY;
	uh.pReg = UART4;

	USART_PCLKControl(&uh, ENABLE);
	USART_Init(&uh);

	USART_IRQInterruptConfig(52, ENABLE);
	USART_IRQPriorityConfig(52, 5);

}

int main()
{
	gpio_init();
	usart_int_init();
	uint8_t ptx[] = "Please make this uppercase!";
	uint8_t prx[] = "                                              ";


	USART_SendDataIT(&uh,ptx, sizeof(ptx));
	USART_ReceiveDataIT(&uh, prx, sizeof(prx));


	while(1);
	return 1;
}

void UART4_IRQHandler()
{
	USART_IRQHandling(&uh);
}
