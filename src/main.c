/**
  ******************************************************************************
  * @file    Templates/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.3
  * @date    06-May-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include "main.h"
#include "uart.h"
#include "i2c.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WHO_AM_I                    0x0F

/* ---------------------------------- Temperature --------------------------- */
#define TEMP_ADDR_WRITE             0xBE
#define TEMP_ADDR_READ              0xBF

#define TEMP_WHO_AM_I               0x0F
#define TEMP_OUT_L                  0x2A
#define TEMP_OUT_H                  0x2B
#define TEMP_T0_OUT_L               0x3C
#define TEMP_T0_OUT_H               0x3D
#define TEMP_T1_OUT_L               0x3E
#define TEMP_T1_OUT_H               0x3F

/* ---------------------------------- Temperature --------------------------- */
#define MAGNETO_ADDR_WRITE		    0x3D
#define MAGNETO_ADDR_READ           0x3C

#define MAGNETO_WHO_AM_I            0x0F

/* ---------------------------------- Pressure ------------------------------ */
#define PRESSURE_ADDR_WRITE         0xBA
#define PRESSURE_ADDR_READ		    0xBB

#define PRESSURE_WHO_AM_I		    0x0F
#define PRESSURE_PRESS_OUT_XL       0x28
#define PRESSURE_PRESS_OUT_L        0x29
#define PRESSURE_PRESS_OUT_H        0x2A

/* ---------------------------------- Accelerometer ------------------------- */
#define ACCELEROMETER_ADDR_WRITE	0xD6
#define ACCELEROMETER_ADDR_READ		0xD7

#define ACCELEROMETER_WHO_AM_I		0x0F

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void wait_us(uint32_t us);
static void wait_ms(uint32_t us);
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

#define N 3

void print_tab(uint8_t *tab, uint32_t size) {
    int i;
    printf("Result: ");
    for(i=0;i<size;i++) {
        printf("\t%X", tab[i]);
    }
    printf("\r\n");
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main() {
    /* configure clocks to use the external Xtal with frequ = 84000000 Hz*/
    set_clk();
	NVIC_SetPriority(USART2_IRQn,5);
	NVIC_EnableIRQ(USART2_IRQn);
	UART2_Init();
    I2C_Init();
    uint8_t data[N] = {};
	/* main application */
    printf("Init over !\r\n");
    for(;;) {
        I2C_Receive(I2C1, ACCELEROMETER_ADDR_READ, ACCELEROMETER_ADDR_WRITE, WHO_AM_I, data, N);
        print_tab(data, N);
        wait_ms(500);
    }
    return 1;
}

/* __io_putchar is called by _write for printf, her il calls UART_Transmit */
void __io_putchar (int ch) {
	uint8_t c;
	c = (uint8_t) ch;
	UART_Transmit(USART2, &c, 1) ;
}

static void wait_us(uint32_t us){
#ifdef DEBUG
	us =(SystemCoreClock/1000000/8)*us;
#else
	us =(SystemCoreClock/1000000/4)*us;
#endif
	while(us--);
}

static void wait_ms(uint32_t ms){
#ifdef DEBUG
	ms =(SystemCoreClock/1000/8)*ms;
#else
	ms =(SystemCoreClock/1000/4)*ms;
#endif
	while(ms--);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
