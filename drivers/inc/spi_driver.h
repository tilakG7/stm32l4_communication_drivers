/*
 * stm32l47x_spi_driver.h
 *
 *  Created on: Sep 4, 2020
 *      Author: tilak
 */

#ifndef SRC_STM32L47X_SPI_DRIVER_H_
#define SRC_STM32L47X_SPI_DRIVER_H_

#include "stm32l4xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_ClkSpeed;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_DFF;
	uint8_t SPI_SSM;

} SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t 	*pTX;
	uint8_t		*pRX;
	uint32_t 	txLen;
	uint32_t 	rxLen;
	uint8_t		txState;
	uint8_t 	rxState;
} SPI_Handle_t;


/*
 * SPI Enums
 */
typedef enum
{
	SPI_FLAG_RXNE 	= 0,
	SPI_FLAG_TXE 	= 1,
	SPI_FLAG_CRCERR = 4,
	SPI_FLAG_MODF 	= 5,
	SPI_FLAG_OVR 	= 6,
	SPI_FLAG_BSY	= 7,
	SPI_FLAG_FRE	= 8,

}SPI_StatusFlag_t;



/*
 * SPI Initializing/Deinitializing
 */
void SPI_SetPCLK(SPI_Handle_t *pSpiHandle, uint8_t enOrDi);
void SPI_Init(SPI_Handle_t *pSpiHandle);
void SPI_EnDi(SPI_Handle_t *pSpiHandle, uint8_t enOrDi);
void SPI_Deinit(SPI_Handle_t *pSpiHandle);

/*
 * SPI Communication
 */
void SPI_RX(SPI_Handle_t *pSpiHandle, uint8_t *pRxBuff, uint32_t buffLen);
void SPI_TX(SPI_Handle_t *pSpiHandle, uint8_t *pTxBuff, uint32_t buffLen);

uint8_t SPI_RX_IT(SPI_Handle_t *pSpiHandle, uint8_t *pRxBuff, uint32_t buffLen);
uint8_t SPI_TX_IT(SPI_Handle_t *pSpiHandle, uint8_t *pTxBuff, uint32_t buffLen);

/*
 * SPI Interrupt Handling
 */
void SPI_SetInterrupt(uint8_t irqNum, uint8_t enOrDi);
void SPI_SetIRQPriority(uint8_t irqNum, uint8_t pri);
void SPI_HandleIRQ(SPI_Handle_t *pSpiHandle);

/*
 * SPI Helper functions
 */
uint8_t SPI_GetStatus(SPI_RegDef_t *pSpiRegDef, SPI_StatusFlag_t flag);
static void SPI_Handle_RXNE(SPI_Handle_t *pSpiHandle);
static void SPI_Handle_TXE(SPI_Handle_t *pSpiHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSpiHandle, uint8_t event);


/*
 * SPI MACROS
 */
#define SPI_DEVICE_MODE_MASTER 			1
#define SPI_DEVICE_MODE_SLAVE			0

#define SPI_BUS_CONFIG_FULL_DUPLEX		0
#define SPI_BUS_CONFIG_HALF_DUPLEX		1
#define SPI_BUS_CONFIG_SIMPLEX_RX		2

#define SPI_SPEED_PS_2					0
#define SPI_SPEED_PS_4					1
#define SPI_SPEED_PS_8					2
#define SPI_SPEED_PS_16					3
#define SPI_SPEED_PS_32					4
#define SPI_SPEED_PS_64					5
#define SPI_SPEED_PS_128				6
#define SPI_SPEED_PS_256				7

#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1

#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

#define SPI_DFF_8						0
#define SPI_DFF_16						1

#define SPI_SSM_DI						0
#define SPI_SSM_EN						1


/*
 * SPI STATE
 */
#define SPI_STATE_READY					0
#define SPI_STATE_IN_RX					1
#define SPI_STATE_IN_TX					2


/*
 * SPI EVENTS
 */
#define SPI_EVENT_TX_COMPLETE			1
#define SPI_EVENT_RX_COMPLETE			2
#define SPI_EVENT_ERR					3


#endif /* SRC_STM32L47X_SPI_DRIVER_H_ */
