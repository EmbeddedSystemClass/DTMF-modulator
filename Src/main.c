/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sinus.h"
#include "keyBoard.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static short * sinOnePointer;
static short * sinTwoPointer;

unsigned short sinOneLength;
unsigned short sinTwoLength;

unsigned short sinOneLicznik;
unsigned short sinTwoLicznik;

volatile bool debug = false;


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		//if
		if (debug == false)
		{
			if (sinOneLength > 0 && sinTwoLength > 0)
			{
				short wynik = 0;

				sinOneLicznik ++;
				if (sinOneLicznik > sinOneLength ) sinOneLicznik = 0;

				sinTwoLicznik ++;
				if (sinTwoLicznik > sinTwoLength ) sinTwoLicznik = 0;

				wynik = sinOnePointer[sinOneLicznik] + sinTwoPointer[sinTwoLicznik];

				//wynik = sinTwoPointer[sinTwoLicznik];
				wynik /= 2;
				wynik += 180;

				TIM1->CCR1 = wynik;
			}
			else
			{
				TIM1->CCR1 = 0;
			}

		}
		else
		{
			if (sinOneLength > 0)
			{
				short wynik = 0;
				sinOneLicznik ++;
				if (sinOneLicznik > sinOneLength ) sinOneLicznik = 0;
				wynik = sinOnePointer[sinOneLicznik];
				wynik += 180;
				TIM1->CCR1 = wynik;
			}
			else
			{
				TIM1->CCR1 = 0;
			}

		}
	}
}


char DataToUart[50];
uint16_t UartSize = 0;
bool uartFlag = false;


void ButtonDown(uint8_t nrButton)
{
	if (debug == false)
	{
		switch (nrButton)
		{
		case 1:
			sinOnePointer = sin697;
			sinOneLength = sin697_len;

			sinTwoPointer = sin1209;
			sinTwoLength = sin1209_len;
			break;
		case 2:
			sinOnePointer = sin697;
			sinOneLength = sin697_len;

			sinTwoPointer = sin1336;
			sinTwoLength = sin1336_len;
			break;
		case 3:
			sinOnePointer = sin697;
			sinOneLength = sin697_len;

			sinTwoPointer = sin1477;
			sinTwoLength = sin1477_len;
			break;
		case 4:
			sinOnePointer = sin697;
			sinOneLength = sin697_len;

			sinTwoPointer = sin1633;
			sinTwoLength = sin1633_len;
			break;
		case 5:
			sinOnePointer = sin770;
			sinOneLength = sin770_len;

			sinTwoPointer = sin1209;
			sinTwoLength = sin1209_len;
			break;
		case 6:
			sinOnePointer = sin770;
			sinOneLength = sin770_len;

			sinTwoPointer = sin1336;
			sinTwoLength = sin1336_len;
			break;
		case 7:
			sinOnePointer = sin770;
			sinOneLength = sin770_len;

			sinTwoPointer = sin1477;
			sinTwoLength = sin1477_len;
			break;
		case 8:
			sinOnePointer = sin770;
			sinOneLength = sin770_len;

			sinTwoPointer = sin1633;
			sinTwoLength = sin1633_len;
			break;
		case 9:
			sinOnePointer = sin852;
			sinOneLength = sin852_len;

			sinTwoPointer = sin1209;
			sinTwoLength = sin1209_len;
			break;
		case 10:
			sinOnePointer = sin852;
			sinOneLength = sin852_len;

			sinTwoPointer = sin1336;
			sinTwoLength = sin1336_len;
			break;
		case 11:
			sinOnePointer = sin852;
			sinOneLength = sin852_len;

			sinTwoPointer = sin1477;
			sinTwoLength = sin1477_len;
			break;
		case 12:
			sinOnePointer = sin852;
			sinOneLength = sin852_len;

			sinTwoPointer = sin1633;
			sinTwoLength = sin1633_len;
			break;
		case 13:
			sinOnePointer = sin941;
			sinOneLength = sin941_len;

			sinTwoPointer = sin1209;
			sinTwoLength = sin1209_len;
			break;
		case 14:
			sinOnePointer = sin941;
			sinOneLength = sin941_len;

			sinTwoPointer = sin1336;
			sinTwoLength = sin1336_len;
			break;
		case 15:
			sinOnePointer = sin941;
			sinOneLength = sin941_len;

			sinTwoPointer = sin1477;
			sinTwoLength = sin1477_len;
			break;
		case 16:
			sinOnePointer = sin941;
			sinOneLength = sin941_len;

			sinTwoPointer = sin1633;
			sinTwoLength = sin1633_len;
			break;
		}
		sinOneLicznik = 0;
		sinTwoLicznik = 0;
	}
	else
	{
		if ( nrButton == 1) { sinOnePointer = sin697; sinOneLength = sin697_len; }
		else if (nrButton == 2) { sinOnePointer = sin770; sinOneLength = sin770_len; }
		else if (nrButton == 3) { sinOnePointer = sin852; sinOneLength = sin852_len; }
		else if (nrButton == 4) { sinOnePointer = sin941; sinOneLength = sin941_len; }
		else if (nrButton == 5) { sinOnePointer = sin1209; sinOneLength = sin1209_len; }
		else if (nrButton == 6) { sinOnePointer = sin1336; sinOneLength = sin1336_len; }
		else if (nrButton == 7) { sinOnePointer = sin1477; sinOneLength = sin1477_len; }
		else if (nrButton == 8) { sinOnePointer = sin1633; sinOneLength = sin1633_len; }

		sinOneLicznik = 0;
		sinTwoLicznik = 0;
	}


	UartSize = sprintf(DataToUart, "Down: %d\n\r", nrButton);
	uartFlag = true;
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

}


void ButtonUp(uint8_t nrButton)
{
	sinOneLength = 0;
	sinTwoLength = 0;
}

void SysTick_MyHandler()
{
	eachInterrupt();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);

  SetColumns(C0_GPIO_Port, C0_Pin, C1_GPIO_Port, C1_Pin, C2_GPIO_Port, C2_Pin, C3_GPIO_Port, C3_Pin);
  SetRows(R0_GPIO_Port, R0_Pin, R1_GPIO_Port, R1_Pin, R2_GPIO_Port, R2_Pin, R3_GPIO_Port, R3_Pin);
  SetCallbacks(&ButtonDown, &ButtonUp);

  TIM1->CCR1 = 180;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  if (uartFlag == true)
	  	  {
	  		  uartFlag = false;
	  		HAL_UART_Transmit (&huart1, (uint8_t *)DataToUart, UartSize, 10);
	  	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 359;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, R0_Pin|R1_Pin|R2_Pin|R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R0_Pin R1_Pin R2_Pin R3_Pin */
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C0_Pin C1_Pin C2_Pin C3_Pin */
  GPIO_InitStruct.Pin = C0_Pin|C1_Pin|C2_Pin|C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
