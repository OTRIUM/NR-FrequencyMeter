/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#define LED_LOW           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define LED_HIGH          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define CS_T_LOW          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)  
#define CS_T_HIGH         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define CS_F_LOW          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)  
#define CS_F_HIGH         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define PULSEOUT_LOW      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)  
#define PULSEOUT_HIGH     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t freq = 0, temp = 0, stepCounter = 0, outState = 0, extiCounter = 0, count = 0, approxCounts = 0;
uint8_t SELED_Out[5] = {0};

const char simbol[] = 
{
  0xBE,                                                                         // 0
  0x06,                                                                         // 1
  0xDC,                                                                         // 2
  0xCE,                                                                         // 3
  0x66,                                                                         // 4
  0xEA,                                                                         // 5
  0xFA,                                                                         // 6
  0x86,                                                                         // 7
  0xFE,                                                                         // 8
  0xEE,                                                                         // 9
  0x00,                                                                         // NULL
  0x01                                                                          // Dot
};

const float polinom[] = 
{
  52.31562632,                                                                  // Y
  -10.35172256,                                                                 // X1
  1.207717029,                                                                  // X2
  -0.05790674,                                                                  // X3
  0.001131226                                                                   // X4
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void delay(void)                                                                // 75 uS delay                                                                                  
{	
  for (int i = 0; i < 750; i++)
    asm("NOP");
}

void STLED_Init(void)
{
  CS_F_LOW;
  CS_T_LOW;
  delay();
  uint8_t initData = 0x0D;
  HAL_SPI_Transmit(&hspi2, &initData, 1, 0x1000);                               // Display ON
  CS_F_HIGH;
  CS_T_HIGH;
  delay();
  
  CS_F_LOW;
  CS_T_LOW;
  delay();
  initData = 0x10;                                                              
  HAL_SPI_Transmit(&hspi2, &initData, 1, 0x1000);                               // Dimming setting
  initData = 0xFC;
  HAL_SPI_Transmit(&hspi2, &initData, 1, 0x1000);                               // 5 digits, MAX brightness
  CS_F_HIGH;
  CS_T_HIGH;
  delay();
}

void number2array (uint32_t inData, uint8_t outData[5])
{
  if (inData < 10)
  {
    outData[0] = simbol[10];
    outData[1] = simbol[10];
    outData[2] = simbol[0] | simbol[11];
    outData[3] = simbol[0];
    outData[4] = simbol[inData];
  }
  else if (inData < 100)
  {
    outData[0] = simbol[10];
    outData[1] = simbol[10];
    outData[2] = simbol[0] | simbol[11];
    outData[3] = simbol[inData / 10];
    outData[4] = simbol[inData % 10];
  }
  else if (inData < 1000)
  {
    outData[0] = simbol[10];
    outData[1] = simbol[10];
    outData[2] = simbol[inData / 100] | simbol[11];
    while (inData >= 100)
      inData -= 100;
    outData[3] = simbol[inData / 10];
    outData[4] = simbol[inData % 10];    
  }
  else if (inData < 10000)
  {
    outData[0] = simbol[10];
    outData[1] = simbol[inData / 1000];
    while (inData >= 1000)
      inData -= 1000;
    outData[2] = simbol[inData / 100] | simbol[11];
    while (inData >= 100)
      inData -= 100;
    outData[3] = simbol[inData / 10];
    outData[4] = simbol[inData % 10];
  }
  else 
  {
    outData[0] = simbol[inData / 10000];
    while (inData >= 10000)
      inData -= 10000;
    outData[1] = simbol[inData / 1000];
    while (inData >= 1000)
      inData -= 1000;
    outData[2] = simbol[inData / 100] | simbol[11];
    while (inData >= 100)
      inData -= 100;
    outData[3] = simbol[inData / 10];
    outData[4] = simbol[inData % 10];
  }
  
  
}

void STLED_SendData(uint32_t tempData, uint32_t freqData)
{
  /*
  Data out:
  **********
  | xxx.x *C  |
  | xxx.x kHz |
  **********
  tempData = temp * 100 [*C];
  freqData = freq * 100 [kHz];
  */  
  
  uint8_t sendData;
  
  // Temperature Data Output
      
  CS_T_LOW;
  
  delay();
  sendData = 0x00;
  HAL_SPI_Transmit(&hspi2, &sendData, 1, 0x1000);                               // Address

  number2array(tempData, SELED_Out);
  
  HAL_SPI_Transmit(&hspi2, &SELED_Out[0], 1, 0x1000);                             // Dig0
  HAL_SPI_Transmit(&hspi2, &SELED_Out[1], 1, 0x1000);                             // Dig1
  HAL_SPI_Transmit(&hspi2, &SELED_Out[2], 1, 0x1000);                             // Dig2
  HAL_SPI_Transmit(&hspi2, &SELED_Out[3], 1, 0x1000);                             // Dig3
  HAL_SPI_Transmit(&hspi2, &SELED_Out[4], 1, 0x1000);                             // Dig4

  CS_T_HIGH;
  
  // Frequency Data Output
      
  CS_F_LOW;
  
  delay();
  sendData = 0x00;
  HAL_SPI_Transmit(&hspi2, &sendData, 1, 0x1000);                               // Address

  number2array(freqData, SELED_Out);
  
  HAL_SPI_Transmit(&hspi2, &SELED_Out[0], 1, 0x1000);                             // Dig0
  HAL_SPI_Transmit(&hspi2, &SELED_Out[1], 1, 0x1000);                             // Dig1
  HAL_SPI_Transmit(&hspi2, &SELED_Out[2], 1, 0x1000);                             // Dig2
  HAL_SPI_Transmit(&hspi2, &SELED_Out[3], 1, 0x1000);                             // Dig3
  HAL_SPI_Transmit(&hspi2, &SELED_Out[4], 1, 0x1000);                             // Dig4

  CS_F_HIGH;
}   
  
  
/*
  // Temperature Data Output
  CS_T_LOW;
  CS_F_LOW;
  
  delay();
  uint8_t sendData = 0x00;
  HAL_SPI_Transmit(&hspi2, &sendData, 1, 0x1000);                               // Address

  sendData = simbol[1];
  HAL_SPI_Transmit(&hspi2, &sendData, 1, 0x1000);                               // Dig0
  sendData = simbol[4];
  HAL_SPI_Transmit(&hspi2, &sendData, 1, 0x1000);
  sendData = simbol[8];
  HAL_SPI_Transmit(&hspi2, &sendData, 1, 0x1000);
  sendData = simbol[8];
  HAL_SPI_Transmit(&hspi2, &sendData, 1, 0x1000);
  sendData = simbol[9];
  HAL_SPI_Transmit(&hspi2, &sendData, 1, 0x1000);                               // Dig4
  
  CS_T_HIGH;
  CS_F_HIGH;
*/  
  
/*
uint32_t freq2temp(uint32_t freqData)
{
  uint32_t tempData = 0;
  
  if (freqData < 1122)
    tempData = (int) freqData * 1.850;          // 20 - 
  else if (freqData < 1163) 
    tempData = (int) freqData * 1.872;          // 21
  else if (freqData < 1205) 
    tempData = (int) freqData * 1.892;          // 22
  else if (freqData < 1246) 
    tempData = (int) freqData * 1.909;          // 23
  else if (freqData < 1287) 
    tempData = (int) freqData * 1.926;          // 24
  else if (freqData < 1328) 
    tempData = (int) freqData * 1.943;          // 25
  else if (freqData < 1369) 
    tempData = (int) freqData * 1.958;          // 26
  else if (freqData < 1411) 
    tempData = (int) freqData * 1.972;          // 27
  else if (freqData < 1452) 
    tempData = (int) freqData * 1.984;          // 28
  else if (freqData < 1493) 
    tempData = (int) freqData * 1.997;          // 29
  else if (freqData < 1534) 
    tempData = (int) freqData * 2.009;          // 30
  else if (freqData < 1575) 
    tempData = (int) freqData * 2.021;          // 31
  else if (freqData < 1617) 
    tempData = (int) freqData * 2.032;          // 32
  else if (freqData < 1658) 
    tempData = (int) freqData * 2.041;          // 33
  else if (freqData < 1700) 
    tempData = (int) freqData * 2.051;          // 34        
  else if (freqData < 1740) 
    tempData = (int) freqData * 2.059;          // 35
  else if (freqData < 1781)             
    tempData = (int) freqData * 2.069;          // 36
  else if (freqData < 1823) 
    tempData = (int) freqData * 2.077;          // 37         
  else if (freqData < 1864) 
    tempData = (int) freqData * 2.084;          // 38
  else if (freqData < 1905) 
    tempData = (int) freqData * 2.092;          // 39
  else
    tempData = (int) freqData * 2.100;          // 40 +        
  
  return tempData;
}
*/

uint32_t freq2temp(uint32_t freqData)
{
  uint32_t tempData = 0;
  float tempFloat = 0;
  float freqFloat = freqData;
  freqFloat /= 100;
  tempFloat = polinom[1] * freqFloat + polinom[2] * freqFloat * freqFloat + polinom[3] * freqFloat * freqFloat * freqFloat + polinom[4] * freqFloat * freqFloat * freqFloat * freqFloat + polinom[0];
  tempFloat *= 100;
  tempData = (int) tempFloat;
  return tempData;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  
  LED_HIGH;
  STLED_Init();
  HAL_Delay(50);
//  PULSEOUT_HIGH;
  
  STLED_SendData(0,0);

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim3);  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
                                                      
  /* USER CODE END WHILE */
  
  /* USER CODE BEGIN 3 */

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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65534;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 17999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PULSE_OUT_Pin|SPI2_CS_F_Pin|SPI2_CS_T_Pin|LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PULSE_OUT_Pin SPI2_CS_F_Pin SPI2_CS_T_Pin LED0_Pin */
  GPIO_InitStruct.Pin = PULSE_OUT_Pin|SPI2_CS_F_Pin|SPI2_CS_T_Pin|LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

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
