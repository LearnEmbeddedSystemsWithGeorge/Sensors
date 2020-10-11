/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum eedges {rising_Edge, falling_Edge, unknown_Edge} edges_E;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HC_SR04_ECHOCAPTURE_RESET 		0U
#define HC_SR04_ECHOCAPTURE_COMPLETE 	1U
#define HC_SR04_US_TIMER				TIM3 // used for microsecond delay function
//#define HC_SR04_USE_FLOAT //use floating point to calculate the distance
#define HC_SR04_UART_BUFFER_SIZE		100U // 100 characters
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t ecFlag 						= HC_SR04_ECHOCAPTURE_RESET;
edges_E edgeDirection 				= rising_Edge;
uint32_t risingEdgeCounterValue		= 0;
uint32_t fallingEdgeCounterValue	= 0;

char uartBuffer[HC_SR04_UART_BUFFER_SIZE] = {0};

#ifdef HC_SR04_USE_FLOAT
/* Speed of sound 0.0343 centimeter / microsecond, we divide it by 2
 * because of the time the signal is sent out half of the time and reflected back the other half
 */
const float speedOfSoundAir = 0.0343 / 2;
float distance = 0.0;
#else
// Speed of sound in us multiplied by 1024 to not work with floating variables is 35 (0.0343 * 1024 = 35.1232)
const uint8_t speedOfSoundAir = 35;
uint16_t distance = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void DelayMicroseconds(uint32_t uSec);
void Read_Distance(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//Read distance
	Read_Distance();

	// Delay the readings by 1 second
	HAL_Delay(1000); // 1000ms = 1s
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Trigger_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* *
 * Functions taken from MYaqoobEmbedded channel
 * https://www.youtube.com/channel/UC-CuJ6qKst9-8Z-EXjoYK3Q/videos
 * */
void DelayMicroseconds(uint32_t uSec)
{
	uint32_t luSec = 0;

	if(uSec < 2u)
	{
		luSec = 2;
	}
	else
	{
		luSec = uSec;
	}

	HC_SR04_US_TIMER->ARR = luSec - 1; 	//sets the value in the auto-reload register
	HC_SR04_US_TIMER->EGR = 1; 			//Re-initializes the timer
	HC_SR04_US_TIMER->SR &= ~1; 			//Resets the flag
	HC_SR04_US_TIMER->CR1 |= 1; 			//Enables the counter
	while((HC_SR04_US_TIMER->SR&0x0001) != 1);
	HC_SR04_US_TIMER->SR &= ~(0x0001);
}

/* Callback function used for capture, it will overwrite the weak function
 * __weak void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) from stm32f4xx_hal_tim.c
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// Toggle led to see that the capture is happening
	HAL_GPIO_TogglePin(GPIOA, LD2_Pin);

	// First call of the interrupt on rising edge is with edgeDirection = 0
	if(edgeDirection == rising_Edge) //First edge
	{
		// First captured value
		risingEdgeCounterValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		edgeDirection = falling_Edge; // set next edge direction
	}
	// Second call of the interrupt on falling edge is with edgeDirection = 1
	else if(edgeDirection == falling_Edge) //Second edge
	{
		// Second captured value
		fallingEdgeCounterValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		edgeDirection = rising_Edge; // set next edge direction
		// Indicate that the width of pulse was measured
		ecFlag = HC_SR04_ECHOCAPTURE_COMPLETE;
	}
}

void Read_Distance(void)
{
	//0. Set trigger pin to low for a short time, to be sure is low
	HAL_GPIO_WritePin(GPIOA, Trigger_Pin, GPIO_PIN_RESET);
	DelayMicroseconds(3);

	//1. Trigger 8 pulse burst
	HAL_GPIO_WritePin(GPIOA, Trigger_Pin, GPIO_PIN_SET);
	DelayMicroseconds(10);
	HAL_GPIO_WritePin(GPIOA, Trigger_Pin, GPIO_PIN_RESET);

	//2. Measure ECHO signal pulse width
	//Start IC timer
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	//Wait for IC flag
	uint32_t startTick = HAL_GetTick();
	do
	{
		if(ecFlag) break; // ecFlag is set on falling edge of ECHO pin
	}while((HAL_GetTick() - startTick) < 500); // 500ms Timeout to break from while

	ecFlag = HC_SR04_ECHOCAPTURE_RESET; // Reset ecFlag for next measurement
	//Stop IC timer
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);

	//Calculate distance in cm
	if(fallingEdgeCounterValue > risingEdgeCounterValue)
	{
		#ifdef HC_SR04_USE_FLOAT
			distance = ((fallingEdgeCounterValue - risingEdgeCounterValue) + 0.0f) * speedOfSoundAir;
		#else
			// Calculate distance using bit operation, distance will have a lower resolution than floating points
			// When you shift left you multiply with 2^shift value
			// When you shift right you divide by 2^shift value
			distance = ((((fallingEdgeCounterValue - risingEdgeCounterValue) * speedOfSoundAir) >> 10u) >> 1u);
		#endif
	}
	else
	{
		#ifdef HC_SR04_USE_FLOAT
			distance = 0.0;
		#else
			distance = 0;
		#endif
	}

	//Print to UART terminal for debugging
#ifdef HC_SR04_USE_FLOAT
	// Create string to send over UART
	sprintf(uartBuffer, "Distance = %.1fcm, %ld, %ld \r\n", distance, fallingEdgeCounterValue, risingEdgeCounterValue);
#else
	// Create string to send over UART
	sprintf(uartBuffer, "Distance = %dcm, %ld, %ld \r\n", distance, fallingEdgeCounterValue, risingEdgeCounterValue);
#endif
	// Print distance over UART, timeout 100ms
	HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), 100);
}
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
