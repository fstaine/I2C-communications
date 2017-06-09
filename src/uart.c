#include "uart.h"

#define TX_LOCK 1
#define RX_LOCK 2

struct UART_Device uart2 = {
	.instance = USART2,
	.state = 0,
};

void UART2_Init(void)
{
	GPIO_TypeDef * gpioa = GPIOA;
	RCC_TypeDef * rcc = RCC;
	USART_TypeDef * uart = USART2;

	/*-------------- clock enable --------------- */
	/* Enable GPIOA clock */
	rcc->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	/* Enable USARTx clock */
	rcc->APB1ENR |= RCC_APB1ENR_USART2EN;
	/* enable DMA1 clock */
	rcc->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	/* reset USART2 */
	rcc->APB1RSTR |= RCC_APB1RSTR_USART2RST;
	rcc->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

	/*-------------- Pin muxing configuration --------------- */
	/* TX on PA2 alternate function 7 */
	gpioa->AFR[0] &= ~(0xF << (2 *4) );	/* clear the 4 bits */
	gpioa->AFR[0] |= (7 << (2*4) ); 	/* set alternate function */
	/* RX on PA3 alternate function 7 */
	gpioa->AFR[0] &= ~(0xF << (3*4) );	/* clear the 4 bits */
	gpioa->AFR[0] |= (7 << (3*4) );		/* set alternate function */

	/* Configure alternate function for PA2 and PA3*/
	gpioa->MODER &= ~GPIO_MODER_MODER3;
	gpioa->MODER &= ~GPIO_MODER_MODER2;
	gpioa->MODER |= GPIO_MODER_MODER3_1;
	gpioa->MODER |= GPIO_MODER_MODER2_1;

	/*-------------- UART parameters configuration --------------- */
	/* USART CR1 Configuration */
	uart->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8;	/* tx and rx enable; oversampling = 8 */
	/* USART CR2 Configuration */
	uart->CR2 = 0 ; /* 1 stop bit */
	/* USART CR3 Configuration */
	uart->CR3 = 0; /* no flow control */
	/* USART BRR Configuration */
	uart->BRR = 0x2D5; /*45,625 with fpclk= 42E6*/
	/* enable USART */
	uart->CR1 |= USART_CR1_UE;

	/*-------------- variable intialisation --------------- */
	uart2.state &= ~(TX_LOCK | RX_LOCK);
}


uint32_t UART_Transmit(USART_TypeDef * uart, uint8_t * data, uint32_t len)
{
	/* test the lock */
	if ((uart2.state & TX_LOCK) == TX_LOCK)
		return -1;
	/* get the lock*/
	uart2.state &= TX_LOCK;


	while (len-->0){
		/* wait for TX data register to be empty */
		while (!(uart->SR & USART_SR_TXE)){
		}
		uart->DR = *data++;
	}
	/* wait last char to be finished (optionnal) */
	while (!(uart->SR & USART_SR_TC));

	/*release the lock */
	uart2.state &= TX_LOCK;

	return len;
}

uint32_t UART_Receive(USART_TypeDef * uart, uint8_t * data, uint32_t len, uint32_t timeout)
{
	uint32_t dec;

	/* test the lock */
	if ((uart2.state & RX_LOCK) == RX_LOCK)
		return -1;
	/* get the lock*/
	uart2.state &= RX_LOCK;

	while (len > 0){
		dec = timeout;
		/* wait for a new char */
		while (!(uart->SR & USART_SR_RXNE)) {
				if (timeout-- == 0)
					return len;
		}
		/* get the data */
		*data++ = (uart->DR & 0xFF);
		len--;
	}

	/*release the lock */
	uart2.state &= RX_LOCK;

	return len;
}

uint32_t UART_Transmit_IT(USART_TypeDef * uart, uint8_t * data, uint32_t len)
{
	uint32_t ret;

	/* test the lock */
	if ((uart2.state & TX_LOCK) == TX_LOCK)
		return -1;
	uart2.state &= TX_LOCK;

	/* critical section to modify the shared data */
	NVIC_DisableIRQ(USART2_IRQn);

	 /* set the transmission  info  */
	uart2.state = TX_LOCK; 	/* lock */
	uart2.pTxBuffer = data;
	uart2.TxSize=len;
	uart2.TxCount = 0;
	uart2.TxComplete = 0;
	/* enable USART interrupt */
	uart->CR1 |= USART_CR1_TXEIE;
	NVIC_EnableIRQ(USART2_IRQn);

	/* wait for transmission comlete (debug) */
	for(;;) {
		if (uart2.TxComplete == 1){
			ret = uart2.TxCount;
			uart2.state &= ~TX_LOCK;
			break;
		}
		else
			/* let's sleep */
			__WFI();
	}
	return ret;
}

void USART2_IRQHandler(void)
{
	uint32_t status, cr1;

	/* get status register */
	status = uart2.instance->SR;
	/* get Interrupt sources */
	cr1 = uart2.instance->CR1 & 0x1F0;

	/* TXIE interrupt and there's a transmission */
	if ((status & USART_SR_TXE) && (cr1 & USART_CR1_TXEIE) ){
		if (uart2.TxCount < uart2.TxSize){
			uart2.instance->DR = *uart2.pTxBuffer++;
			uart2.TxCount++;
		}
		else {
			/* stop TXE interrupt not needed anymore */
			uart2.instance->CR1 &= ~USART_CR1_TXEIE;
			/* enable TC interrupt to wait for the last char to completely transmitted */
			uart2.instance->CR1 |= USART_CR1_TCIE;
		}
	}

	/* TCIE interrupt and there's a transmission */
	if ((status & USART_SR_TC) && (cr1 & USART_CR1_TCIE)){
		/* stop TXE interrupt */
		uart2.instance->CR1 &= ~USART_CR1_TCIE;

		/* signal end of transmission to the waiting function if any */
		/* a callback function can be used */
		UART_TX_complete_callback();
	}


	/* RXNE interrupt and there's a reception */
	/* ... */
}


void UART_TX_complete_callback(void)
{
	/* signal end of transmission */
	uart2.TxComplete = 1;
}
