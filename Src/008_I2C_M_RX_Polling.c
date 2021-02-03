/*
 * Code to TX Data to Arduino over SPI1 Peripheral
 *
 * Date Created: September 7th, 2020
 * Author:		 Tilak Gupta
 *
 */

#include "i2c_driver.h"
#include "stm32l47x_gpio_driver.h"

#include "string.h"

uint8_t buff[32];
GPIO_Handle_t errLed;

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

I2C_Handle_t i2c_init()
{
	I2C_Config_t config = {.deviceAddress = 0x61};
	I2C_Comm_t   comm   = {.slaveAddMode = 0, .slaveAdd=0x68, .readNotWrite=0, .buffLen=sizeof(buff), .pBuff=buff};
	I2C_Handle_t h = {.config = config, .commConfig = comm, .pI2Cx = I2C1 };

	I2C_PeriClkEnDi(&h, ENABLE);

	return h;
}

void I2CAppEvCallBack(uint8_t evCode)
{
	// turn on error led
	GPIO_WritePin(GPIOB, errLed.GPIO_PinConfig.GPIO_PinNumber, SET);
	while(1);
}



int main()
{
	GPIO_Handle_t button = gpio_button_init();
	gpio_init();
	gpio_led_init();


	while(GPIO_ReadPin(button.pGPIOx, button.GPIO_PinConfig.GPIO_PinNumber) != RESET);
	delay();

	I2C_Handle_t i2cH = i2c_init();
	buff[0] = 0x51; // command to get length of
	i2cH.commConfig.pBuff = buff;
	i2cH.commConfig.buffLen = 1;
	i2cH.commConfig.readNotWrite = 0;
	I2C_Init( &i2cH );
	I2C_Master_TXRXPolling(&i2cH);
	i2cH.commConfig.pBuff = buff;
	i2cH.commConfig.readNotWrite = 1;
	i2cH.commConfig.buffLen = 1;
	I2C_Master_TXRXPolling(&i2cH);

	uint8_t dataLen = buff[0];
	buff[0] = 0x52; // command to read string
	i2cH.commConfig.pBuff = buff;
	i2cH.commConfig.readNotWrite = 0;
	i2cH.commConfig.buffLen = 1;
	I2C_Master_TXRXPolling(&i2cH);

	i2cH.commConfig.pBuff = buff;
	i2cH.commConfig.readNotWrite = 1;
	i2cH.commConfig.buffLen = dataLen;
	I2C_Master_TXRXPolling(&i2cH);




	while(1);
}



