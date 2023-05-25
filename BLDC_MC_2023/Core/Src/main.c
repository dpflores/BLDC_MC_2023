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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Definiciones principales

// Definiciones adicionales

#define COMMUTATION_DELAY_US 150u	// microsegundos para el delay

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// HEADERS DE FUNCIONES

// funciones principales
void get_adc(void);
void read_hall(void);
void bldc_move(void);


//funciones adicionales
void delay_us (uint16_t us);


// VARIABLES
uint16_t raw_adc = 0;

int hall_a = 0;
int hall_b = 0;
int hall_c = 0;

int bldc_step = 0;
int duty_cycle = 0;


// timer flags

uint8_t timer2_flag = 0;


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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */



  // Inicializamos los timer timers
  HAL_TIM_Base_Start(&htim1);	// Timer de PWMs (10kH)
  HAL_TIM_Base_Start_IT(&htim2);	// Timer principal (2.5 kHz)
  HAL_TIM_Base_Start(&htim3);		// Timer para delay de microsegundos

  // Inicializamos los 3 canales de PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // Iniciamos con 0 de duty los 3 PWM HIGH de loS MOSFETS
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  // Inicializamos en low las salidas LOW de los MOSFETS
  HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);

  // Incializamos el led de testeo apagado
  HAL_GPIO_WritePin(TEST_LED_GPIO_Port , TEST_LED_Pin,  GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (timer2_flag == 1){
		  get_adc();

		  read_hall();

		  bldc_move();

		  timer2_flag = 0;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 8-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 800-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A_LOW_Pin|B_LOW_Pin|C_LOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TEST_LED_Pin */
  GPIO_InitStruct.Pin = TEST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_A_Pin HALL_B_Pin HALL_C_Pin */
  GPIO_InitStruct.Pin = HALL_A_Pin|HALL_B_Pin|HALL_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : A_LOW_Pin B_LOW_Pin C_LOW_Pin */
  GPIO_InitStruct.Pin = A_LOW_Pin|B_LOW_Pin|C_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//FUNCIONES

void get_adc(void){
	// Obtenemos la data del ADC
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	raw_adc = HAL_ADC_GetValue(&hadc1);

	duty_cycle = raw_adc/41;	// Obtiene el valor de entre 0 a 100 para el duty

	// Para que el sistema se suelte cuando el deseado sea menos del 10%
	if (duty_cycle<10){
		duty_cycle = 0;
	}
}

void read_hall(void){
	hall_a = HAL_GPIO_ReadPin(HALL_A_GPIO_Port,HALL_A_Pin);
	hall_b = HAL_GPIO_ReadPin(HALL_B_GPIO_Port,HALL_B_Pin);
	hall_c = HAL_GPIO_ReadPin(HALL_C_GPIO_Port,HALL_C_Pin);

	// Uncomment after testing
	bldc_step = hall_a + 2*hall_b + 4*hall_c;

	// testing for one value
//	bldc_step = 3;
	// end testing

	// testing for continous changes
//	if (bldc_step > 6){
//		bldc_step = 1;
//	}
//	else {
//		bldc_step++;
//	}
	// end testing

}

void bldc_move(void){

	switch(bldc_step){
	case 1:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);	//B_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//C_HIGH
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);

		// Delay
		delay_us(COMMUTATION_DELAY_US);


		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);	//A_HIGH
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_SET);
		break;
	case 2:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//A_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//C_HIGH
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);

		// Delay
		delay_us(COMMUTATION_DELAY_US);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_cycle);	//B_HIGH
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_SET);

		break;
	case 3:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//A_HIGH

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//C_HIGH

		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);


		// Delay
		delay_us(COMMUTATION_DELAY_US);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_cycle);	//B_HIGH
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_SET);

		break;
	case 4:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//A_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);	//B_HIGH
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);

		// Delay
		delay_us(COMMUTATION_DELAY_US);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_cycle);	//C_HIGH
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_SET);

		break;

	case 5:

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);	//B_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//C_HIGH

		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);

		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);

		// Delay
		delay_us(COMMUTATION_DELAY_US);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);	//A_HIGH
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_SET);
		break;

	case 6:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//A_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);	//B_HIGH



		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);

		// Delay
		delay_us(COMMUTATION_DELAY_US);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_cycle);	//C_HIGH
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_SET);

		break;

	default:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//A_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);	//B_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//C_HIGH

		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);
		break;
	}
}

void delay_us(uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim3,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // wait for the counter to reach the us input in the parameter
}
//INTERRUPCIONES

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	// Verificamos si es el timer principal y ejecutamos todo
	if (htim == &htim2){

		timer2_flag = 1;

	}
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
