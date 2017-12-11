/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */

#define PULSEOUT_LOW      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)  
#define PULSEOUT_HIGH     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

#define __HAL_TIM_GetCounter(__HANDLE__) ((__HANDLE__)->Instance->CNT)

extern uint32_t stepCounter, outState, extiCounter, freq, temp, count, approxCounts;
uint32_t i = 0, k = 0, ii = 0;
uint32_t IC_period [300] = {0};
uint32_t period [300] = {0};

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles ADC1 and ADC2 global interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  
  /* USER CODE END TIM2_IRQn 0 */
  
  HAL_TIM_IRQHandler(&htim2);
  
  /* USER CODE BEGIN TIM2_IRQn 1 */
  
  /* USER CODE END TIM2_IRQn 1 */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  
  IC_period[i] = __HAL_TIM_GetCounter(htim);
  htim->Instance->CNT = 0;
  if (i<100)
    i++;
  else 
    i = 0;
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  
  stepCounter++;                                                                // step 5 mS
  
  if (stepCounter == 1)                                                         // 0 mS
  {
    PULSEOUT_HIGH;
    outState = 1;
    extiCounter = 0;
    i = 0;
  }

  else if (stepCounter == 2)                                                    // 5 mS
  {
    PULSEOUT_LOW;
  
    if (i > 2)                                                                  // Overflow check
    {
      ii = i;
      count = ii - 2;

      for (k=0; k<300; k++)
        period[k] = 0;
   
      approxCounts = 0;

      for (k=1; k<=count; k++)
      {
        period[k] = IC_period[k];
        approxCounts += period[k];
      }
      
      approxCounts /= count;
        
      for (ii=0; ii<300; ii++)
        IC_period[ii] = 0; 
    }
  }
  else if (stepCounter == 51)                                                   // 250 mS
  {
    if (approxCounts != 0)
    {
      freq = 360 * 10000 / approxCounts;
      temp = freq2temp(freq);
    }
    else
    {
      freq = 0;
      temp = 0;
    }
    STLED_SendData(temp, freq);
  }
  else if (stepCounter == 401)                                                  // 2000 mS
    stepCounter = 0;
  
  
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HardFault_Handler(void){
  while(1);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
