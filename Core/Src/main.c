/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../inc/P10.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
volatile uint16_t i = 0;
volatile uint16_t k = 0;
uint8_t pData[16];
Pixel pix[8];
//координаты для передней стенки
uint16_t front_lu_angle_x = 10;
uint16_t front_lu_angle_y = 10;

uint16_t front_ru_angle_x = 16;
uint16_t front_ru_angle_y = 10;

uint16_t front_rd_angle_x = 16;
uint16_t front_rd_angle_y = 4;

uint16_t front_ld_angle_x = 10;
uint16_t front_ld_angle_y = 4;

//координаты для задней стенки
uint16_t back_lu_angle_x = 16;
uint16_t back_lu_angle_y = 10;

uint16_t back_ru_angle_x = 22;
uint16_t back_ru_angle_y = 10;

uint16_t back_rd_angle_x = 22;
uint16_t back_rd_angle_y = 4;

uint16_t back_ld_angle_x = 16;
uint16_t back_ld_angle_y = 4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void part_0_0() {
	HAL_GPIO_WritePin(GPIOE, nOE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, SCLK_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) pData, 16, 10);
	HAL_GPIO_WritePin(GPIOE, SCLK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, nOE_Pin, GPIO_PIN_SET);
}

void part_0_1() {
	HAL_GPIO_WritePin(GPIOE, nOE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, SCLK_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) pData, 16, 10);
	HAL_GPIO_WritePin(GPIOE, SCLK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, nOE_Pin, GPIO_PIN_SET);
}

void part_1_0() {
	HAL_GPIO_WritePin(GPIOE, nOE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, SCLK_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) pData, 16, 10);
	HAL_GPIO_WritePin(GPIOE, SCLK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, nOE_Pin, GPIO_PIN_SET);
}

void part_1_1() {
	HAL_GPIO_WritePin(GPIOE, nOE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, SCLK_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) pData, 16, 10);
	HAL_GPIO_WritePin(GPIOE, SCLK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, nOE_Pin, GPIO_PIN_SET);
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//вершины передней стенки
		pix[0].x = (front_lu_angle_x + i + k);
		pix[0].y = (front_lu_angle_y - i + k);
		pix[1].x = (front_ru_angle_x - i - k);
		pix[1].y = (front_ru_angle_y + i - k);
		pix[2].x = (front_rd_angle_x - i - k);
		pix[2].y = (front_rd_angle_y + i - k);
		pix[3].x = (front_ld_angle_x + i + k);
		pix[3].y = (front_ld_angle_y - i + k);
		//вершины задней стенки
		pix[4].x = (back_lu_angle_x + i + k);
		pix[4].y = (back_lu_angle_y - i + k);
		pix[5].x = (back_ru_angle_x - i - k);
		pix[5].y = (back_ru_angle_y + i - k);
		pix[6].x = (back_rd_angle_x - i - k);
		pix[6].y = (back_rd_angle_y + i - k);
		pix[7].x = (back_ld_angle_x + i + k);
		pix[7].y = (back_ld_angle_y - i + k);
//передняя стенка
//1
		clearScreen((uint8_t*) &pData);
		drawLine(pix[0], pix[1], (uint8_t*) &pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[0], pix[1], (uint8_t*) &pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[0], pix[1], (uint8_t*) &pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[0], pix[1], (uint8_t*) &pData, P10AB_00);
		part_0_0();
//2
		clearScreen((uint8_t*) &pData);
		drawLine(pix[1], pix[2], (uint8_t*) &pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[1], pix[2], (uint8_t*) &pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[1], pix[2], (uint8_t*) &pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[1], pix[2], (uint8_t*) &pData, P10AB_00);
		part_0_0();
//3
		clearScreen((uint8_t*) &pData);
		drawLine(pix[2], pix[3], (uint8_t*) &pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[2], pix[3], (uint8_t*) &pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[2], pix[3], (uint8_t*) &pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[2], pix[3], (uint8_t*) &pData, P10AB_00);
		part_0_0();
//4
		clearScreen((uint8_t*) &pData);
		drawLine(pix[0], pix[3], (uint8_t*) &pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[0], pix[3], (uint8_t*) &pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[0], pix[3], (uint8_t*) &pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) &pData);
		drawLine(pix[0], pix[3], (uint8_t*) &pData, P10AB_00);
		part_0_0();

//задняя стенка
//1
		clearScreen((uint8_t*) pData);
		drawLine(pix[4], pix[5], (uint8_t*) pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[4], pix[5], (uint8_t*) pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) pData);
		drawLine(pix[4], pix[5], (uint8_t*) pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[4], pix[5], (uint8_t*) pData, P10AB_00);
		part_0_0();
//2
		clearScreen((uint8_t*) pData);
		drawLine(pix[5], pix[6], (uint8_t*) pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[5], pix[6], (uint8_t*) pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) pData);
		drawLine(pix[5], pix[6], (uint8_t*) pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[5], pix[6], (uint8_t*) pData, P10AB_00);
		part_0_0();
//3
		clearScreen((uint8_t*) pData);
		drawLine(pix[6], pix[7], (uint8_t*) pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[6], pix[7], (uint8_t*) pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) pData);
		drawLine(pix[6], pix[7], (uint8_t*) pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[6], pix[7], (uint8_t*) pData, P10AB_00);
		part_0_0();
//4
		clearScreen((uint8_t*) pData);
		drawLine(pix[7], pix[4], (uint8_t*) pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[7], pix[4], (uint8_t*) pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) pData);
		drawLine(pix[7], pix[4], (uint8_t*) pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[7], pix[4], (uint8_t*) pData, P10AB_00);
		part_0_0();

//соединения
//1
		clearScreen((uint8_t*) pData);
		drawLine(pix[0], pix[4], (uint8_t*) pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[0], pix[4], (uint8_t*) pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) pData);
		drawLine(pix[0], pix[4], (uint8_t*) pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[0], pix[4], (uint8_t*) pData, P10AB_00);
		part_0_0();
//2
		clearScreen((uint8_t*) pData);
		drawLine(pix[1], pix[5], (uint8_t*) pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[1], pix[5], (uint8_t*) pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) pData);
		drawLine(pix[1], pix[5], (uint8_t*) pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[1], pix[5], (uint8_t*) pData, P10AB_00);
		part_0_0();
//3
		clearScreen((uint8_t*) pData);
		drawLine(pix[2], pix[6], (uint8_t*) pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[2], pix[6], (uint8_t*) pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) pData);
		drawLine(pix[2], pix[6], (uint8_t*) pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[2], pix[6], (uint8_t*) pData, P10AB_00);
		part_0_0();
//4
		clearScreen((uint8_t*) pData);
		drawLine(pix[3], pix[7], (uint8_t*) pData, P10AB_11);
		part_1_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[3], pix[7], (uint8_t*) pData, P10AB_10);
		part_1_0();
		clearScreen((uint8_t*) pData);
		drawLine(pix[3], pix[7], (uint8_t*) pData, P10AB_01);
		part_0_1();
		clearScreen((uint8_t*) pData);
		drawLine(pix[3], pix[7], (uint8_t*) pData, P10AB_00);
		part_0_0();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 3000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60000-1;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, nOE_Pin|A_Pin|SCLK_Pin|B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : but_Pin */
  GPIO_InitStruct.Pin = but_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(but_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nOE_Pin A_Pin SCLK_Pin B_Pin */
  GPIO_InitStruct.Pin = nOE_Pin|A_Pin|SCLK_Pin|B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {

  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
