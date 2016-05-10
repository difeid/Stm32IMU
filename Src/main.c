/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "stm32f3_discovery.h"
#include "stm32f3_discovery_gyroscope.h"
#include "stm32f3_discovery_magnitometer.h"
#include "stm32f3_discovery_accelerometer.h"

#include "imu_util.h"
#include "MadgwickAHRS.h"
#include "MadgwickFullAHRS.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t UserButtonPressed = 0;
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void ReadSensors(float *g, float *a, float *m);
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t tb1, tb2;
  float samplePeriod;
  float g[3], a[3], m[3];
  float quaternion[4] = { 1.0f, .0f, .0f, .0f };
  float ypr[3] = {.0f};
  float speed[2] = { .0f };
  float per[2] = { .0f };
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  // MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  BSP_GYRO_Init();
  BSP_ACCELERO_Init();
  BSP_MAGNITO_Init();
  BSP_LED_Init(LED_GREEN);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  tb1 = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      ReadSensors(g, a, m);

      tb2 = HAL_GetTick();
      samplePeriod = (float)(0.001f * (tb2 - tb1));
      tb1 = tb2;

      switch(UserButtonPressed)
	{
	case 0:
	  {
	    printf (
		"G %7.3f,%7.3f,%7.3f\nA %7.3f,%7.3f,%7.3f\nM %7.3f,%7.3f,%7.3f\n\n",
		g[0], g[1], g[2],
		a[0], a[1], a[2],
		m[0], m[1], m[2]);
	    break;
	  }
	case 1:
	  {
	    for(uint8_t i = 0; i < 2; i++)
	      {
		a[i] = a[i] * 9.81f;
	        speed[i] += a[i] * samplePeriod;
	        per[i] += speed[i] * samplePeriod;
	      }
	    printf("S %5.1f,%5.1f P %5.1f,%5.1f\n",
		   speed[0], speed[1], per[0], per[1]);
	    break;
	  }
	case 2:
	  {
	    imuDegToRadV3 (g);
	    MadgwickAHRSupdate (g, a, m, samplePeriod, quaternion);
	    /*printf("Q2 %6.3f,%6.3f,%6.3f,%6.3f,%3dHz\n",
		   quaternion[0], quaternion[1], quaternion[2], quaternion[3],
		   (int)(1.0f / samplePeriod));*/
	    imuQuaternionToYawPitchRoll(quaternion, ypr);
	    imuRadToDegV3(ypr);
	    printf("YPR %7.1f,%7.1f,%7.1f,%4dHz\n", ypr[0], ypr[1], ypr[2], (int)(1.0f / samplePeriod));
	    break;
	  }
	case 3:
	  {
	    imuDegToRadV3(g);
	    MadgwickFullAHRSUpdate(g, a, m, samplePeriod, quaternion);
	    /*printf("Q3 %6.3f,%6.3f,%6.3f,%6.3f,%3dHz\n",
		   quaternion[0], quaternion[1], quaternion[2], quaternion[3],
		   (int)(1.0f / samplePeriod));*/
	    imuQuaternionToYawPitchRoll(quaternion, ypr);
	    imuRadToDegV3(ypr);
	    printf("YPR %7.1f,%7.1f,%7.1f%,4dHz\n", ypr[0], ypr[1], ypr[2], (int)(1.0f / samplePeriod));
	    break;
	  }
	case 4:
	  {
	    imuNormalizeV3(a);
	    imuNormalizeV3(m);
	    printf("PGH%5.1f,%5.1f,%7.1f,%7.1f\n",
		   RAD2DEG(imuPitch(a[0], a[1], a[2])),
		   RAD2DEG(imuRoll(a[0], a[1], a[2])),
		   RAD2DEG(imuHeading(m[0], m[1], m[2])),
		   RAD2DEG(imuHeadingTiltCompensated(m[0], m[1], m[2], a[0], a[1], a[2])));
	    break;
	  }
	default:
	  UserButtonPressed = 0;
	}

      //BSP_LED_Toggle(LED_GREEN);
      //HAL_Delay(10);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB6   ------> I2C1_SCL
     PB7   ------> I2C1_SDA
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA5 PA6 SPI1_MISO_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|SPI1_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ReadSensors(float *g, float *a, float *m)
{
  float buffer[3] = {.0f};

  BSP_GYRO_GetXYZ(buffer);
  g[0] = buffer[1];
  g[1] = buffer[0];
  g[2] = -buffer[2];

  BSP_ACCELERO_GetXYZ(buffer);
  a[0] = buffer[0];
  a[1] = -buffer[1];
  a[2] = buffer[2];

  BSP_MAGNITO_GetXYZ(buffer);
  m[0] = -buffer[0];
  m[1] = buffer[1];
  m[2] = -buffer[2];
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==USER_BUTTON_PIN)
  {
    UserButtonPressed++;
    if (UserButtonPressed > 4)
    {
      UserButtonPressed = 0;
    }
  }
}

// printf to UART (syscalls.c)
void tty_outc( char ch )
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1);
}

char tty_inc()
{
  char ch = 'e';
  //HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 1);
  return ch;
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
