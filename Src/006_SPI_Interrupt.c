/*
 * Test SPI TX with Interrupts API.
 * Code to TX Data to Arduino over SPI1 Peripheral
 *
 * Date Created: September 7th, 2020
 * Author:		 Tilak Gupta
 *
 */

#include "spi_driver.h"
#include "gpio_driver.h"

#include "string.h"


SPI_Handle_t spi;
char txBuff[] = " Arduino are you reading this?";

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
void spi_it_init()
{


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

	// 2. Enable interrupts
	SPI_SetInterrupt(35, ENABLE); // enable irq#35
	SPI_SetIRQPriority(35, 5);	  // set priority of irq#35 to 5
}




int main()
{

	GPIO_Handle_t nssH    = gpio_init();
	GPIO_Handle_t buttonH = gpio_button_init();
	spi_it_init();


	while(GPIO_ReadPin(GPIOC, GPIO_PIN_NO_13) != RESET);
	delay();


	// Interrupt setup
	txBuff[0] = 23; // set the first byte to indicate length for arduino
	SPI_EnDi(&spi, ENABLE);
	while(SPI_TX_IT(&spi, (uint8_t *)txBuff, strlen(txBuff) + 1) != SPI_STATE_READY);




	while(1);
}

void SPI1_IRQHandler()
{
	SPI_HandleIRQ(&spi);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSpiHandle, uint8_t event)
{
	if(event == SPI_EVENT_TX_COMPLETE)
	{
		SPI_EnDi(&spi, DISABLE);
		SPI_Deinit(&spi);
	}
}


