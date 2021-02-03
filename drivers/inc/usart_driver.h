#ifndef INC_USART_DRIVER_H_
#define INC_USART_DRIVER_H_

#include "stm32l4xx.h"

/* -----------------------------------------------------------------------*/
/* --------------------------- USART Definitions -------------------------*/
/* -----------------------------------------------------------------------*/
/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_7BITS  0
#define USART_WORDLEN_8BITS  1
#define USART_WORDLEN_9BITS  2


/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_RTS    	1
#define USART_HW_FLOW_CTRL_CTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3




/*
 * USART Flag Names
 */
#define USART_FLAG_PE		0
#define USART_FLAG_FE		1
#define USART_FLAG_NF		2
#define USART_FLAG_ORE		3
#define USART_FLAG_IDLE		4
#define USART_FLAG_RXNE		5
#define USART_FLAG_TC		6
#define USART_FLAG_TXE		7
#define USART_FLAG_LBDF		8
#define USART_FLAG_CTSIF	9
#define USART_FLAG_CTS		10
#define USART_FLAG_RTOF		11
#define USART_FLAG_EOBF		12
#define USART_FLAG_ABRE		14
#define USART_FLAG_ABRF		15
#define USART_FLAG_BUSY		16
#define USART_FLAG_CMF		17
#define USART_FLAG_SBKF		18
#define USART_FLAG_RWU		19
#define USART_FLAG_WUF		20
#define USART_FLAG_TEACK	21
#define USART_FLAG_REACK	22
#define USART_FLAG_TCGBT	25


/*
 * USART State
 */
#define USART_STATE_READY	0
#define USART_STATE_RX		1
#define USART_STATE_TX		2

/* -----------------------------------------------------------------------*/
/* ---------------------------- USART Structures -------------------------*/
/* -----------------------------------------------------------------------*/

typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoStopBits;
	uint8_t USART_DataWordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

typedef struct
{
	USART_Config_t c;
	USART_RegDef_t *pReg;
	uint8_t rxState;
	uint8_t txState;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t txBufferLen;
	uint8_t rxBufferLen;
}USART_Handle_t;






/* ----------------------------------------------------------------------*/
/* ----------------------------- USART APIs -----------------------------*/
/* ----------------------------------------------------------------------*/

/*
 * Peripheral Clock setup
 */
void USART_PCLKControl(USART_Handle_t *p, uint8_t enOrDi);


/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);



#endif
