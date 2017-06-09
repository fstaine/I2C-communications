#ifndef __UART_H
#define __UART_H


#include <stm32f4xx.h>

/* Structure */

struct UART_Device {
	USART_TypeDef  * instance;	/* address of the USART registers */
	uint32_t state	;		/* can be used as a lock for transmission and reception, error, ... */
	uint8_t * pTxBuffer;		/* application buffer for transmission */
	uint8_t * pRxBuffer;		/* application buffer for reception */
	uint32_t TxSize;		/* number of data to transmit */
	uint32_t Rxsize;		/* number of data to receive */
	uint32_t TxCount;		/* number of data already transmitted */
	uint32_t RxCount;		/* number of data already received */
	uint32_t TxComplete;	/* signal the end of transmission */
};

/* prototype */
void UART2_Init(void);
uint32_t UART_Transmit(USART_TypeDef * uart, uint8_t * data, uint32_t len);
uint32_t UART_Receive(USART_TypeDef * uart, uint8_t * data, uint32_t len, uint32_t timeout);
void UART_TX_complete_callback(void);
uint32_t UART_Transmit_IT(USART_TypeDef * uart, uint8_t * data, uint32_t len);

#endif /* __UART_H */
