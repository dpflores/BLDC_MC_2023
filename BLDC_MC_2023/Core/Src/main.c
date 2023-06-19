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
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Definiciones principales
#define POLE_PAIRS 23u
#define VOLTAJE 48u
#define STEP_ADC_RPM 10u
#define ESTIMATION_RATE 5u // solo puede ser divisor de 10 {1, 2, 5, 10}.

#define STEPS2RPM 60*ESTIMATION_RATE/POLE_PAIRS/6 // 60 segundos, 2 Hz, 23 pp, 6 pasos
#define RPM2KMH 1/10.44
#define SPEED_UNITS 0u		// 0 for RPM, 1 for KMH
#define TELEMETRY 0u		  // 0 for normal operation, 1 for telemetry
#define PID 0u				// 0 for normal operation, 1 for PID longitudinal control

#define LONGITUDINAL_CAN_ID 255u  // ID decimal para la trama del control longitudinal
#define MC_CAN_ID 259u            // ID decimal de este dispositivo (Motor Controller)
#define HMI_CAN_ID 1u             // ID decimal del display del volante
// Definiciones adicionales

#define COMMUTATION_DELAY_US 350u	// microsegundos para el delay


/* Controller parameters */
// Para ESTIMATION_RATE de 2u: kp = 0.2; ki = 0.8; kd=0.0 (delay 700u)
//						   5u: kp = 0.02; ki = 0.5; kd=0.0 (delay 700u)
#define PID_KP  0.1f// 0.05f //
#define PID_KI  0.6f// 0.5f // 0.15 0.01
#define PID_KD  0.0f //0.0

#define PID_TAU 0.02f

#define PID_LIM_MIN  0.0f
#define PID_LIM_MAX  100.0f

#define SAMPLE_TIME_S 0.1f


/* CAN data */

//Position of the respective data flags

#define GF 0u  	// General flag
#define PW 1u	// Power flag
#define EM 2u	// Emergency flag
#define SU 3u	// Speed unit		0 para RPM, 1 para KMH
#define CS0 4u	// Cruise Speed 0
#define CS1 5u	// Cruise Speed 1



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// HEADERS DE FUNCIONES

// funciones principales
void get_adc(void);
void read_hall(void);
void bldc_move(void);
void bldc_move_back(void);

//funciones adicionales
void delay_us (uint16_t us);
void send_can_data(void);

// VARIABLES
uint8_t power = 0;
uint16_t raw_adc = 0;
uint16_t rate_adc = 0;

uint8_t hall_a = 0;
uint8_t hall_b = 0;
uint8_t hall_c = 0;

uint8_t direction = 0;		// 0 for forward, 1 for backward (ALTAS CORRIENTES EN Backward)
uint8_t bldc_prev_step = 0;
uint8_t bldc_step = 0;
uint16_t steps = 0;

float desired_speed_rpm = 0;
float current_speed_rpm = 0;
float error = 0; //difference between desired and measured speed
float integral = 0;
uint8_t duty_cycle = 0;


// timer flags
uint8_t timer2_flag = 0;
uint8_t timer4_flag = 0;
uint8_t timer4_counts = 0;
uint8_t estimation_flag = 0;



//CAN VARIABLES
uint8_t CAN_FLAG_REG;	// Register of flags for can
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;
uint32_t FreeMailbox; // Check the amount of mailboxes free
uint8_t TxData[8];
uint8_t RxData[8];


//PID
PIDController pid = { PID_KP, PID_KI, PID_KD, PID_TAU,
						PID_LIM_MIN, PID_LIM_MAX, SAMPLE_TIME_S };


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
  MX_TIM4_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

	
  // Inicializamos el PID
  PIDController_Init(&pid);

  // Inicializamos los timer timers
  HAL_TIM_Base_Start(&htim1);	// Timer de PWMs (10kH)
  HAL_TIM_Base_Start_IT(&htim2);	// Timer principal (2.5 kHz)
  HAL_TIM_Base_Start(&htim3);		// Timer para delay de microsegundos
  HAL_TIM_Base_Start_IT(&htim4);	// Timer para el control (10Hz)

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


  //CAN
  HAL_CAN_Start(&hcan);
  //CAN FIFO activation
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	//Configurando la transmision
  TxHeader.DLC = 8;  // Son 8 bytes de data
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD; //Identificador del mensaje
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = MC_CAN_ID;  // Este es el ID que mandaremos al periferico (revisar defines)
  TxHeader.TransmitGlobalTime = DISABLE;

  // CAN_FLAG_REG Initialization
  CAN_FLAG_REG |=  (1<<GF);	// General Flag 1 (necessary)
  CAN_FLAG_REG = (CAN_FLAG_REG & (~(1 << SU))) | (SPEED_UNITS << SU); // speed unit setting a
  CAN_FLAG_REG &= ~(1<<EM); // No Emergency
  CAN_FLAG_REG &= ~(1<<PW); // Initial OFF
  CAN_FLAG_REG &= ~(1<<CS0); //
  CAN_FLAG_REG &= ~(1<<CS1); // Cruise speed



  TxData[0] = 0; 	//Speed component
  TxData[1] = 0;	//Speed component
  TxData[2] = 0;	//Duty Cycle
  TxData[3] = 0;	// ...
  TxData[4] = 0;	// ...
  TxData[5] = 0;	// ...
  TxData[6] = 0;	// ...
  TxData[7] = CAN_FLAG_REG;	//Flags byte  [0 0 0 0 0 0 0 1] [x x cs cs su em pw gf]


  // x: not defined
  // cs: cruice speed (0 to 3)
  // su: speed units (0 for RPM, 1 for km/h)
  // em: emergency ( 1 for alert)
  // pw: Power flag (0 off, 1 on)
  // gf: general flag (1 always)



  // Incializamos el led de testeo apagado
  HAL_GPIO_WritePin(TEST_LED_GPIO_Port , TEST_LED_Pin,  GPIO_PIN_SET);

  // Leemos los hall por primera vez
  read_hall();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (timer2_flag == 1){
      // Si se activa la telemetria la velocidad deseada llega a partir de la data CAN
		  if (!TELEMETRY) {
			  get_adc();
		  }

		  read_hall();

		  if (direction == 0){
			  bldc_move();
		  }
		  else {
			  bldc_move_back();
		  }

		  timer2_flag = 0;

	  }

	  if (timer4_flag == 1 && PID){
		  //CONTROL PID
			uint8_t u = 0;

		  PIDController_Update(&pid, desired_speed_rpm, current_speed_rpm);

		  // Protection
		  if(desired_speed_rpm<=20){
			  PIDController_Reset(&pid);
		  }

			integral = pid.integrator;

			u = pid.out;

			duty_cycle = u;

		  timer4_flag = 0;
	  }

	  if (estimation_flag){

		  // Estimamos y enviamos por CAN la velocidad
		  current_speed_rpm = steps*STEPS2RPM;
		  send_can_data();	// Enviamos la data por CAN
		  estimation_flag = 0;
		  steps = 0;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x01<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x01<<5;	// esto solo permite el id que tiene el bit 1 activo, osea id's impares
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assigned to CAN1

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  /* USER CODE END CAN_Init 2 */

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
  htim2.Init.Period = 400-1;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

//	duty_cycle = raw_adc/41;	// Obtiene el valor de entre 0 a 100 para el duty
//
//	// Para que el sistema se suelte cuando el deseado sea menos del 10%
//	if (duty_cycle<10){
//		duty_cycle = 0;
//	}

	if (PID){
		rate_adc = STEP_ADC_RPM*48*12/VOLTAJE;
		raw_adc = rate_adc*(raw_adc/rate_adc);
		desired_speed_rpm = raw_adc*STEP_ADC_RPM/rate_adc;
	}
	else {
		duty_cycle = 100*raw_adc/4095;
	}
}

void read_hall(void){
	hall_a = HAL_GPIO_ReadPin(HALL_A_GPIO_Port,HALL_A_Pin);
	hall_b = HAL_GPIO_ReadPin(HALL_B_GPIO_Port,HALL_B_Pin);
	hall_c = HAL_GPIO_ReadPin(HALL_C_GPIO_Port,HALL_C_Pin);

	bldc_step = hall_a + 2*hall_b + 4*hall_c;

	// Para calcular velocidad
	if (bldc_step != bldc_prev_step){
		steps += 1;
	}

	bldc_prev_step	= bldc_step;


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

void bldc_move_back(void){

	switch(bldc_step){
	case 6:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);	//B_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//C_HIGH
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);

		// Delay
		delay_us(COMMUTATION_DELAY_US);


		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);	//A_HIGH
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_SET);
		break;
	case 5:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//A_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//C_HIGH
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);

		// Delay
		delay_us(COMMUTATION_DELAY_US);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_cycle);	//B_HIGH
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_SET);

		break;
	case 4:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//A_HIGH

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//C_HIGH

		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);


		// Delay
		delay_us(COMMUTATION_DELAY_US);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_cycle);	//B_HIGH
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_SET);

		break;
	case 3:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//A_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);	//B_HIGH
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);

		// Delay
		delay_us(COMMUTATION_DELAY_US);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_cycle);	//C_HIGH
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_SET);

		break;

	case 2:

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);	//B_HIGH
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//C_HIGH

		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);

		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);

		// Delay
		delay_us(COMMUTATION_DELAY_US);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);	//A_HIGH
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_SET);
		break;

	case 1:
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

void send_can_data(void){

	if (SPEED_UNITS == 0){
		// Current speed (RPM)
		if (current_speed_rpm > 255){
				TxData[0] = 255;
				TxData[1] = current_speed_rpm - 255;
		}
		else{
			TxData[0] = current_speed_rpm;
			TxData[1] = 0;
		}
		// Desired speed (RPM)
		if (desired_speed_rpm > 255){
				TxData[3] = 255;
				TxData[4] = desired_speed_rpm - 255;
		}
		else{
			TxData[3] = desired_speed_rpm;
			TxData[4] = 0;
		}
	}
	else{
		// KMPH
		TxData[0] = current_speed_rpm*RPM2KMH;
		TxData[1] = 0;

	}

	// Alojamos el duty cycle
	TxData[2] = duty_cycle;

	//Enviamos el estado ON OFF del sistema
	CAN_FLAG_REG = (CAN_FLAG_REG & (~(1 << PW))) | (power << PW); // Sending the ON (1) or OFF (0)


	TxData[7] = CAN_FLAG_REG;	// Actualizamos con los flag

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

}

//INTERRUPCIONES

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim == &htim2){

		timer2_flag = 1;

	}
	if (htim == &htim4){

		timer4_flag = 1;

		if (timer4_counts == 10/ESTIMATION_RATE){
			estimation_flag = 1;

			timer4_counts = 0;
		}
		timer4_counts += 1;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

  // Si es que se tiene activada la telemetria y el ID es del axotec
  if (TELEMETRY && RxHeader.StdId == AXOTEC_ID){
	  desired_speed_rpm = RxData[0] + RxData[1];
  }

  // Para la data que llega del HMI
	if (RxData[0] == 96 && RxData[1] == 234){ // HMI Page 60000

		if (RxData[4] == 1){ //HMI button 1
			power = 1;
		}
		if (RxData[4] == 2){ //HMI button 2
			power = 0;
		}


		if (RxData[5] == 16){ //HMI down button
			//TxData[4] = 0;		//Change button color
			CAN_FLAG_REG &= ~(1 << CS1);
			CAN_FLAG_REG &= ~(1 << CS0); //Cruise speed 0

		}
		if (RxData[5] == 8){ //HMI left button
			//TxData[4] = 1; 		//Change button color
			CAN_FLAG_REG &= ~(1 << CS1);
			CAN_FLAG_REG |=  (1 << CS0); //Cruise speed 1

		}
		if (RxData[5] == 32){ //HMI right button
			//TxData[4] = 2;		//Change button color
			CAN_FLAG_REG |=  (1 << CS1);
			CAN_FLAG_REG &= ~(1 << CS0); //Cruise speed 2

		}
		if (RxData[5] == 2){ //HMI up button
			//TxData[4] = 3;		//Change button color
			CAN_FLAG_REG |=  (1 << CS1);
			CAN_FLAG_REG |=  (1 << CS0); //Cruise speed 3

		}

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
