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
#define TEMP_ID         0xBF
#define MAGNETIC_ID     0b00111101 //0b00111101 //0x3D
#define PRESSURE_ID     0b10111101
#define ACCEMLERO_ID    0b01101000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint16_t temp, hum;

/* Private function prototypes -----------------------------------------------*/
static void wait_us(uint32_t us);
static void wait_ms(uint32_t us);
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

#define N 5

void print_tab(uint8_t *tab, uint32_t size) {
    int i;
    printf("Result: ");
    for(i=0;i<size;i++) {
        printf("\t%d", tab[i]);
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
        I2C_Receive(I2C1, MAGNETIC_ID, data, N);
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

// #if USE_HAL_DRIVER
//
// /**
//   * @brief  System Clock Configuration
//   *         The system Clock is configured as follow :
//   *            System Clock source            = PLL (HSI)
//   *            SYSCLK(Hz)                     = 84000000
//   *            HCLK(Hz)                       = 84000000
//   *            AHB Prescaler                  = 1
//   *            APB1 Prescaler                 = 2
//   *            APB2 Prescaler                 = 1
//   *            HSI Frequency(Hz)              = 16000000
//   *            PLL_M                          = 16
//   *            PLL_N                          = 336
//   *            PLL_P                          = 4
//   *            PLL_Q                          = 7
//   *            VDD(V)                         = 3.3
//   *            Main regulator output voltage  = Scale2 mode
//   *            Flash Latency(WS)              = 2
//   * @param  None
//   * @retval None
//   */
// static void SystemClock_Config(void)
// {
//   RCC_ClkInitTypeDef RCC_ClkInitStruct;
//   RCC_OscInitTypeDef RCC_OscInitStruct;
//
//   /* Enable Power Control clock */
//   __HAL_RCC_PWR_CLK_ENABLE();
//
//   /* The voltage scaling allows optimizing the power consumption when the device is
//      clocked below the maximum system frequency, to update the voltage scaling value
//      regarding system frequency refer to product datasheet.  */
//   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
//
//   /* Enable HSI Oscillator and activate PLL with HSI as source */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = 0x10;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//   RCC_OscInitStruct.PLL.PLLM = 16;
//   RCC_OscInitStruct.PLL.PLLN = 336;
//   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//   RCC_OscInitStruct.PLL.PLLQ = 7;
//   if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }
//
//   /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
//      clocks dividers */
//   RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//   if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }
//
// /**
//   * @brief  This function is executed in case of error occurrence.
//   * @param  None
//   * @retval None
//   */
// static void Error_Handler(void)
// {
//   /* User may add here some code to deal with this error */
//   while(1)
//   {
//   }
// }
//
// #ifdef  USE_FULL_ASSERT
// /**
//   * @brief  Reports the name of the source file and the source line number
//   *         where the assert_param error has occurred.
//   * @param  file: pointer to the source file name
//   * @param  line: assert_param error line source number
//   * @retval None
//   */
// void assert_failed(uint8_t* file, uint32_t line)
// {
//   /* User can add his own implementation to report the file name and line number,
//      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//
//   /* Infinite loop */
//   while (1)
//   {
//   }
// }
// #endif
// #endif /* USE_HAL_DRIVER */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
