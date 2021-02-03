/*
 * Code to TX Data to Arduino over SPI1 Peripheral
 *
 * Date Created: September 7th, 2020
 * Author:		 Tilak Gupta
 *
 */

#include "stm32l47x_spi_driver.h"
#include "stm32l47x_gpio_driver.h"

#include "string.h"


void delay()
{
	for(int i = 0; i < 100000; i++);
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
	button.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALT_FCN_5;

	// 1. Init GPIOC Clock
	GPIO_ClockControl(GPIOC, ENABLE);

	// 2. Init GPIO Pin
	GPIO_Init(&button);

	return button;
}

/*
 * Initialize PA4-PA7 for SPI1 functionality
 * Return the handle for the NSS pin (PA4)
 */
GPIO_Handle_t gpio_init()
{
	GPIO_Handle_t gpio;
	gpio.pGPIOx = GPIOA;

	gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFCN;
	gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VHIGH_SPEED;
	gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALT_FCN_5;

	// 1. Enable GPIOA Peripheral Clock
	GPIO_ClockControl(GPIOA, ENABLE);

	// 2. Init PA7 (SPI_MOSI)
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&gpio);

	// 3. Init PA6 (SPI_MISO)
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&gpio);

	// 4. Init PA5 (SPI_SCK)
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&gpio);

	// 5. Init PA4 (SPI_NSS)
	gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&gpio);

	return gpio;
}

/*
 * Initialize SPI1 Peripheral for Full Duplex communication
 */
SPI_Handle_t spi_init()
{
	SPI_Handle_t spi;

	spi.pSPIx = SPI1;

	spi.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	spi.SPI_Config.SPI_ClkSpeed = SPI_SPEED_PS_8;
	spi.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	spi.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	spi.SPI_Config.SPI_DFF = SPI_DFF_8;
	spi.SPI_Config.SPI_SSM = SPI_SSM_DI;

	// 1. Enable SPI1 Clock
	SPI_SetPCLK(&spi, ENABLE);
	SPI_Init(&spi);

	return spi;
}

int main()
{
	GPIO_Handle_t nssH    = gpio_init();
	GPIO_Handle_t buttonH = gpio_button_init();
	SPI_Handle_t  spiH    = spi_init();

	char txBuff[] = "Do you copy Arduino!???";
	uint8_t txBuffLen = strlen(txBuff);

	while(GPIO_ReadPin(GPIOC, GPIO_PIN_NO_13) != RESET);
	delay();
	SPI_EnDi(&spiH, ENABLE);

	SPI_TX(&spiH, &txBuffLen, 1);
	SPI_TX(&spiH, &txBuff, txBuffLen);

	SPI_EnDi(&spiH, DISABLE);
	SPI_Deinit(&spiH);

	while(1);
}
