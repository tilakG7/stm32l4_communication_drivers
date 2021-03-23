/*
 * Test I2C Slave RX/TX Interrupts API.
 * Some code to test I2C data reception as a slave. Uses interrupts, not polling.
 *
 * Date Created: September 7th, 2020
 * Author:		 Tilak Gupta
 *
 */

#include "i2c_driver.h"
#include "gpio_driver.h"

#include "string.h"
#define MY_ADDR 0x69


GPIO_Handle_t errLed;
I2C_Handle_t h;

void delay()
{
	for(int i = 0; i < 100000; i++);
}

void gpio_led_init()
{
	errLed.pGPIOx = GPIOB;
	errLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	errLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	errLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VHIGH_SPEED;
	errLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PU_PD;
	errLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_ClockControl(GPIOB, ENABLE);
	GPIO_Init(&errLed);
}

GPIO_Handle_t gpio_button_init()
{
	GPIO_Handle_t button;
	button.pGPIOx = GPIOC;

	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VHIGH_SPEED;
	button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	//button.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALT_FCN_5;

	// 1. Init GPIOC Clock
	GPIO_ClockControl(GPIOC, ENABLE);

	// 2. Init GPIO Pin
	GPIO_Init(&button);

	return button;
}

/*
 * Initialize GPIO Port B for I2C
 * PB9 - SDA (Alternative Function #4)
 * PB8 - SCL (Alternative Function #4)
 */
GPIO_Handle_t gpio_init()
{
	GPIO_Handle_t gpio;
	gpio.pGPIOx = GPIOB;

	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFCN;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VHIGH_SPEED;
	gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // SDA & SCL need pull up
	gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;  // SDA & SCL require Open Drain
	gpio.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALT_FCN_4;

	// 1. Enable GPIOB Peripheral Clock
	GPIO_ClockControl(GPIOB, ENABLE);

	// 2. Init PB8 I2C1_SCL
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&gpio);

	// 3. Init PB9 (I2C_SDA)
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&gpio);

	return gpio;
}

volatile int transferLenState = -1; // -1: Nothing, 0x52: transferData 0x51: transferData
uint8_t buff[] = "This is some data";
uint8_t len = sizeof(buff);
uint8_t* pSlaveBuff;

void i2c1_init()
{
	I2C_Config_t config = {.deviceAddress = MY_ADDR};
	I2C_Comm_t   comm   = {.slaveAddMode = 0, .slaveAdd=0x68, .readNotWrite=0, .buffLen=sizeof(buff), .pBuff=buff};
	h.config = config;
	h.commConfig = comm;
	h.pI2Cx = I2C1;
	h.state = I2C_STATE_READY;


	I2C_IRQ_Config(31, ENABLE);
	I2C_IRQ_Config(32, ENABLE);
	I2C_IRQ_Priority(31, 5);
	I2C_IRQ_Priority(32, 6);

	I2C_PeriClkEnDi(&h, ENABLE);
}

void I2C_AppEvCallback(uint8_t evCode)
{
	// turn on error led
	if(evCode != I2C_EV_TRANSACTION_SUCCESS)
		GPIO_WritePin(GPIOB, errLed.GPIO_PinConfig.GPIO_PinNumber, SET);
	else
		GPIO_WritePin(GPIOB, errLed.GPIO_PinConfig.GPIO_PinNumber, RESET);
}

void I2C_SlaveReceiveCallback(uint8_t d)
{
	transferLenState = d;
}

uint8_t* I2C_SlaveTransmitCallback()
{
	pSlaveBuff++;
	return (pSlaveBuff - 1);
}


int main()
{
	GPIO_Handle_t button = gpio_button_init();
	gpio_init();
	gpio_led_init();
	i2c1_init();


	I2C_Init(&h);
	uint8_t currState = transferLenState;
	I2C_Slave_TXRX_INT(&h);
	while(1)
	{
		// detect a state change
		if(transferLenState != currState)
		{
			I2C_Slave_TXRX_INT(&h);
			currState = transferLenState;
			if(currState ==  0x51)
			{
				pSlaveBuff = &len;
			}
			else if(currState == 0x52)
			{
				pSlaveBuff = buff;
			}
		}

	}
}

void I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandler(&h);
}

void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandler(&h);
}
