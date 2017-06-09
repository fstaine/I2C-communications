#ifndef __I2C_H
#define __I2C_H

#include <stm32f4xx.h>

/* Structure */
// I2C_TypeDef

typedef struct {
	I2C_TypeDef  * instance;	/* address of the USART registers */
	uint32_t state	;		/* can be used as a lock for transmission and reception, error, ... */
	uint8_t * pTxBuffer;		/* application buffer for transmission */
	uint8_t * pRxBuffer;		/* application buffer for reception */
	uint32_t TxSize;		/* number of data to transmit */
	uint32_t Rxsize;		/* number of data to receive */
	uint32_t TxCount;		/* number of data already transmitted */
	uint32_t RxCount;		/* number of data already received */
	uint32_t TxComplete;	/* signal the end of transmission */
} I2C_Device;

/* prototype */
void I2C_Init(void);
uint32_t I2C_Transmit(I2C_Device * i2c, uint8_t * data, uint32_t len);
uint32_t I2C_Receive(I2C_TypeDef *i2c, uint8_t address, uint8_t *data, uint32_t len);
uint32_t I2C_Receive_IT(I2C_TypeDef *i2c, uint8_t address, uint8_t *data, uint32_t len);
void I2C_TX_complete_callback(void);
uint32_t I2C_Transmit_IT(I2C_Device * i2c, uint8_t * data, uint32_t len);

#endif /* __I2C_H */
