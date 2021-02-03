/*
 * i2c_driver.h
 *
 *  Created on: Sep 10, 2020
 *      Author: tilak
 */

#ifndef INC_I2C_DRIVER_H_
#define INC_I2C_DRIVER_H_

#include "stm32l4xx.h"


/* -----------------------------------------------------------
 *  I2C User Configuration Macros
 * -----------------------------------------------------------
 */

/*
 * @speed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM	400000

/*
 * @ack control
 */
#define I2C_ACK_EN			1
#define I2C_ACK_DI			0

/*
 * duty cycle
 */
#define I2C_FM_DUTY_2 		0	// not sure if my MCU has
#define I2C_FM_DUTY_16_9	1	// not sure if my MCU has


#define I2C_ADD_MODE_7		0
#define I2C_ADD_MODE_10		1

/*
 * i2c states
 */
#define I2C_STATE_READY			0
#define I2C_STATE_IN_RX			1
#define I2C_STATE_IN_TX			2
#define I2C_STATE_SLAVE			3

/* -----------------------------------------------------------
 *  I2C Handle and Configuration Structures
 * -----------------------------------------------------------
 */
typedef struct
{
	uint8_t deviceAddress;
} I2C_Config_t;

typedef struct {
	uint8_t slaveAddMode;	    // 0 indicates 7 bit addressing
	uint8_t slaveAdd; 				// slave address to communicate with
	uint8_t readNotWrite; 			// indicates whether Master is performing a read from or write to slave
	uint32_t buffLen;				// number of bytes remaining to be written or read
	uint8_t *pBuff;
} I2C_Comm_t;

typedef struct
{
	I2C_Config_t config;
	I2C_Comm_t commConfig;
	I2C_RegDef_t *pI2Cx;
	uint8_t state;					// used for interrupts
} I2C_Handle_t;



/* -----------------------------------------------------------
 *  I2C APIs
 * -----------------------------------------------------------
 */
void I2C_PeriClkEnDi(I2C_Handle_t *pi2c, uint8_t enOrDi);
void I2C_Init(I2C_Handle_t *pi2c);
void I2C_Deinit(I2C_Handle_t *pi2c);

/*
 * Polling APIs
 */
void I2C_Master_TXRXPolling(I2C_Handle_t *pi2c);
static void I2C_MasterTXPolling(I2C_Handle_t *pi2c);
static void I2C_MasterRXPolling(I2C_Handle_t *pi2c);


/*
 * Interrupt APIs
 */
uint8_t I2C_MasterTXRX_INT(I2C_Handle_t *pi2c, uint8_t buffLen, uint8_t *pBuff, uint8_t rnw);
void I2C_Slave_TXRX_INT(I2C_Handle_t *pi2c);

void I2C_IRQ_Config(uint8_t irqNum, uint8_t enOrDi);
void I2C_IRQ_Priority(uint8_t irqNum, uint8_t pri);

void I2C_EV_IRQHandler(I2C_Handle_t *pi2c);
void I2C_ER_IRQHandler(I2C_Handle_t *pi2c);



/* -----------------------------------------------------------
 *  I2C Application Callback Functions and Events
 * -----------------------------------------------------------
 */

#define I2C_EV_M_COMM_FAIL  		1// I2C master communication fail event
		//occurs when master fails to control the bus or receive an ACK from the slave address
#define I2C_EV_M_TX_FAIL    		2// I2C master tx fail event. Occurs when NACK is received from slave

#define I2C_EV_M_NACK				3 // I2C master received a NACK
#define I2C_EV_M_ERR				4 // I2C error occured
#define I2C_EV_TRANSACTION_SUCCESS 	5

/*
 * I2C Application Event Callback Function
 * -Desc:
 * 		Callback that needs to be implemented by the application
 * 		to deal with different driver events
* 	@event:
* 		The event that took place that needs to be handled by the application
 */
void I2C_AppEvCallback(uint8_t event);
void I2C_SlaveReceiveCallback(uint8_t d);
uint8_t* I2C_SlaveTransmitCallback();


#endif /* INC_I2C_DRIVER_H_ */
