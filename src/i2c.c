
#include "i2c.h"
#include <stdio.h>

I2C_Device i2c1 = {
	.instance = I2C1,
    .state = 0,
};

int i = 0;
uint8_t data[10] = {};

/*
(p467)
The following is the required sequence in master mode.
    • Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
    • Configure the clock control registers
    • Configure the rise time register
    • Program the I2C_CR1 register to enable the peripheral
    • Set the START bit in the I2C_CR1 register to generate a Start condition
    The peripheral input clock frequency must be at least:
        • 2 MHz in Sm mode
        • 4 MHz in Fm mode
*/

#define I2C_FREQ                42
#define I2C_SPEED               210
#define I2C_RISE_T				43

void I2C_Init(void) {
	GPIO_TypeDef *gpio = GPIOB;
	RCC_TypeDef *rcc = RCC;
    I2C_TypeDef *i2c = I2C1;

    /* Enable I2C1 & GPIOB clock */
    rcc->APB1ENR |=  RCC_APB1ENR_I2C1EN;
	rcc->AHB1ENR |=  RCC_AHB1ENR_GPIOBEN;

    /* Reset I2C1 clock */
    rcc->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
    rcc->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    /* Use alternate function 4 */
    gpio->MODER &= ~GPIO_MODER_MODER8;
    gpio->MODER |=  GPIO_MODER_MODER8_1;
    gpio->AFR[1] &= ~(0xF << (0*4));
    gpio->AFR[1] |= (4 << (0*4));
	gpio->OTYPER |=  (1 << 8);

    gpio->MODER &= ~GPIO_MODER_MODER9;
    gpio->MODER |=  GPIO_MODER_MODER9_1;
    gpio->AFR[1] &= ~(0xF << (1*4));
    gpio->AFR[1] |= (4 << (1*4));
	gpio->OTYPER |=  (1 << 9);

    i2c->CR1 &= ~I2C_CR1_SMBUS;              // i2c mode
    i2c->CR2 |=  (I2C_FREQ & I2C_CR2_FREQ);  // Peripheral clock frequency, <= 0b101010

    i2c->CCR &= ~I2C_CCR_FS;                 // Set as standard mode (not fast)
    i2c->CCR |=  (I2C_SPEED & I2C_CCR_CCR);  // Reset speed
    /* T high = CCR * T PCLK1 --- T low = CCR * T PCLK1 */

	i2c->TRISE &= ~I2C_TRISE_TRISE;
	i2c->TRISE |=  (I2C_RISE_T & I2C_TRISE_TRISE);

    i2c->CR1 |=  I2C_CR1_PE;                 // Peripheral enable
}

uint32_t I2C_Receive(I2C_TypeDef *i2c, uint8_t address, uint8_t *data, uint32_t len) {
	// printf("Starting reception...\r\n");
	i2c->CR1 |=  I2C_CR1_START;
	while(!(i2c->SR1 & I2C_SR1_SB))
		;
	// printf("While 1 terminé\r\n");
	i2c->DR = address;
	while(!(i2c->SR1 & I2C_SR1_ADDR))
		;
	if (i2c->SR2) {
		while(!(i2c->SR1 & I2C_SR1_RXNE))
			;
		*data++ = i2c->DR;
		// printf("Yeah : %X\r\n", i2c->DR);
	}

	// printf("Done\r\n");
}

uint32_t I2C_Receive_IT(I2C_TypeDef *i2c, uint8_t address, uint8_t *data, uint32_t len) {
	i2c1.pRxBuffer = data;
	i2c1.Rxsize = len;
	i2c1.RxCount = 0;

	i2c->CR2 |=  I2C_CR2_ITBUFEN;
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	i2c->CR1 |=  I2C_CR1_START;
}

void I2C_EV_IRQHandler() {
	I2C_TypeDef *i2c = i2c1.instance;
	if (i2c->SR1 & I2C_SR1_RXNE) {
		*i2c1.pRxBuffer++ = i2c->DR;
		i2c1.RxCount++;
	}
}
