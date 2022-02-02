/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "stdlib.h"
#include "string.h"
#include "sensor.h"
#include "pause.h"
#include "ST7920_SERIAL.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PASCALS_TO_MM_CONSTANT 0.00750062
#define TIMCLOCK   100000000
#define PRESCALAR_FOR_TIMER_2  100
#define PRESCALAR_FOR_TIMER_11  40
#define SOLAR_ROTOR_PWM_FREQUENCY 25000
#define  UART_BUFFER_SIZE 20
#define  WEATHER_SENSOR_PARAMETERS_BUFFER_SIZE  20
#define  BUFFER_SIZE 10
#define  SOLAR_ROTOR_RATES_REFERENCE_VALUE 1.5
#define  ROTOR_MAGNETS_AMOUNT  8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
// TaskHandle_t UART_receive_handle;
volatile float frequency = 0;

volatile float previous_frequency = 0;

volatile float RPS = 0;

volatile uint8_t is_stop = 1;

osThreadId UART_receive_handle;

osThreadId  control_motor_task_handle;

SemaphoreHandle_t UART_receive_semaphore_handle;

SemaphoreHandle_t clear_frequency_semaphore_handle;

SemaphoreHandle_t control_motor_semaphore_handle;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void uart_receive_task(void *parameters);

void  control_solar_rotor_task(void *parameters);

// void view(struct DATA_FROM_SENSOR *);
void view();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t input_buffer[UART_BUFFER_SIZE] = {};

struct DATA_FROM_WEATHER_SENSOR data;

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  pause_init();

  ST7920_Init();

  weather_sensor_init();

  /* Start of capturing  rates per second */

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  /* UART for weather sensor */

  HAL_UART_Receive_IT(&huart1, input_buffer, 20);

  /* Start of pwm to switch on our motor   */

  TIM4->CCR1 = 10;

  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);


  /* Start of ADC to measure Current and voltage  */

  HAL_ADCEx_InjectedStart_IT(&hadc1);

  UART_receive_semaphore_handle = xSemaphoreCreateCounting(3, 0);

   if(NULL == UART_receive_semaphore_handle)
   {
 	  printf("can't use semaphore");
   }

   clear_frequency_semaphore_handle = xSemaphoreCreateCounting(1, 0);

   if(NULL == clear_frequency_semaphore_handle)
   {
	   printf("can't use semaphore");
   }

   control_motor_semaphore_handle = xSemaphoreCreateCounting(1, 0);

   if(NULL == control_motor_semaphore_handle)
   {
   	   printf("can't use semaphore");
   }


//   vTaskStartScheduler();


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  osThreadDef(UART_Task, uart_receive_task, osPriorityNormal, 0, 256);
  UART_receive_handle = osThreadCreate(osThread(UART_Task), NULL);

  osThreadDef(Control_Task, control_solar_rotor_task, osPriorityHigh, 0, 256);
  control_motor_task_handle = osThreadCreate(osThread(Control_Task), NULL);


  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = 4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
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
  htim3.Init.Prescaler = 100-1;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 40-1;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void uart_receive_task(void *parameters)
{

	while(1)
	{
		xSemaphoreTake(UART_receive_semaphore_handle, portMAX_DELAY );

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		process(&data, input_buffer, &huart1);

		view();

	}
}

volatile  uint8_t  motor_is_blocked_by_sensor = 0;

void control_solar_rotor_task(void *parameters)
{

	while (1)
	{
		HAL_TIM_ChannelStateTypeDef  channel_state;

		channel_state = HAL_TIM_GetChannelState(&htim4, TIM_CHANNEL_1);

		if ( HAL_TIM_CHANNEL_STATE_BUSY == channel_state )
		{
			if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
			{
				osDelay(10000);

				if ( GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
				{

					HAL_TIM_PWM_Stop_IT (&htim4, TIM_CHANNEL_1);
				}

			}
		}
		else if ( HAL_TIM_CHANNEL_STATE_READY  ==  channel_state )
		{

			if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
			{
				osDelay(5000);

				if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
				{
					TIM4->CCR1 = 10;

					HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
				}

			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, input_buffer, UART_BUFFER_SIZE);

	if (USART1 == huart->Instance)
	{
		if (0x5a == input_buffer[0])
		{
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;

			xSemaphoreGiveFromISR(UART_receive_semaphore_handle, xHigherPriorityTaskWoken);

			portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}

	}

}


void view()
{
	uint8_t  temperature_buffer[WEATHER_SENSOR_PARAMETERS_BUFFER_SIZE];

	uint8_t humidity_buffer[WEATHER_SENSOR_PARAMETERS_BUFFER_SIZE];

	uint8_t  IAQ_buffer[WEATHER_SENSOR_PARAMETERS_BUFFER_SIZE];

	uint8_t pressure_buffer[WEATHER_SENSOR_PARAMETERS_BUFFER_SIZE + 6];

	uint8_t altitude_buffer[WEATHER_SENSOR_PARAMETERS_BUFFER_SIZE];

	uint8_t IAQ_accuracy_buffer[WEATHER_SENSOR_PARAMETERS_BUFFER_SIZE];

	uint16_t milimeters = 0;

	uint8_t clear = 0;

	uint8_t frequency_buffer[WEATHER_SENSOR_PARAMETERS_BUFFER_SIZE];

	ST7920_Clear();

	// memset(temperature_buffer,'\0', sizeof(temperature_buffer));

	snprintf(temperature_buffer , sizeof(temperature_buffer),"%.2f C", data.Temperature);

	ST7920_SendString(0, 0, temperature_buffer);

	// memset(humidity_buffer,'\0', sizeof(humidity_buffer ));

	snprintf(humidity_buffer, sizeof(humidity_buffer),"%.2f %%", data.Humidity);

	ST7920_SendString(1, 0, humidity_buffer);

	// memset(IAQ_buffer,'\0', sizeof(IAQ_buffer ));

	snprintf(IAQ_buffer, sizeof(IAQ_buffer),"%u", data.IAQ);

	ST7920_SendString(2, 0, IAQ_buffer);

	// memset(pressure_buffer,'\0', sizeof(pressure_buffer));

	milimeters = data.Pressure * PASCALS_TO_MM_CONSTANT;

	snprintf(pressure_buffer, sizeof(pressure_buffer),"%u, (%lu)", milimeters, data.Pressure);

	ST7920_SendString(3, 0, pressure_buffer);

	// memset(altitude_buffer,'\0', sizeof(altitude_buffer));

	snprintf(altitude_buffer, sizeof(altitude_buffer),"%i m", data.Altitude);

	ST7920_SendString(0, 5, altitude_buffer);

	// memset(IAQ_accuracy_buffer,'\0', sizeof(IAQ_accuracy_buffer));

	snprintf(IAQ_accuracy_buffer, sizeof(IAQ_accuracy_buffer),"%u", data.IAQ_accuracy);

	ST7920_SendString(2, 5, IAQ_accuracy_buffer);

	snprintf(frequency_buffer, sizeof(frequency_buffer),"%.2f", frequency);

	ST7920_SendString(1, 4, frequency_buffer);

	// memset(frequency_buffer,'\0', sizeof(frequency_buffer));


}


volatile uint32_t ic_value_1 = 0;
volatile uint32_t ic_value_2 = 0;
volatile uint32_t difference = 0;
volatile  int is_first_captured = 0;

/* Measure Frequency */
// float frequency = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (TIM2 == htim->Instance &&   HAL_TIM_ACTIVE_CHANNEL_1 == htim->Channel)
	{
		if (0 == is_first_captured) // if the first rising edge is not captured
		{
			ic_value_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value

			is_first_captured = 1;  // set the first captured as true

		}
		else   // If the first rising edge is captured, now we will capture the second edge
		{
			ic_value_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

			if (ic_value_2 > ic_value_1)
			{
				difference = ic_value_2-ic_value_1;
			}
			else if (ic_value_1 > ic_value_2)
			{
				difference = (0xffffffff - ic_value_1) + ic_value_2;
			}

			float refClock = TIMCLOCK/(PRESCALAR_FOR_TIMER_2);

			frequency = refClock/difference;

			previous_frequency = frequency;

			RPS = frequency / ROTOR_MAGNETS_AMOUNT; // rates per second

			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			is_first_captured = 0; // set it back to false

			BaseType_t xHigherPriorityTaskWoken = pdFALSE;

			xSemaphoreGiveFromISR(clear_frequency_semaphore_handle, xHigherPriorityTaskWoken);

//			xSemaphoreGiveFromISR(control_motor_semaphore_handle, xHigherPriorityTaskWoken);

			portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

			is_stop = 0; // reset stop of motor
		}
	}
}

/* Please refactor it, we can use semaphore, to reduce load  on Interrupt handler */

volatile uint32_t counter = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if ( TIM4 == htim->Instance )
	{
		++counter;

		/* We can change duty cycle of pwm   */

		if(SOLAR_ROTOR_PWM_FREQUENCY == counter)
		{
			if (is_stop)
			{

				TIM4->CCR1 = 1;

			}
			else if(RPS < SOLAR_ROTOR_RATES_REFERENCE_VALUE)
			{

				TIM4->CCR1 = 80;
			}
			else if(RPS > SOLAR_ROTOR_RATES_REFERENCE_VALUE)
			{

				TIM4->CCR1 = 10;
			}

			counter = 0;

			is_stop = 1;

		}

	}

}

/*

volatile uint32_t  first_point_timer_5 = 0;

volatile uint32_t  second_point_timer_5 = 0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if( GPIO_Pin == GPIO_PIN_12) {

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		xSemaphoreGiveFromISR(control_motor_semaphore_handle, xHigherPriorityTaskWoken);

		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

	}

}

*/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Start_IT(hadc);
}

/* This function is not finished */

volatile uint16_t adc[BUFFER_SIZE]={};

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (ADC1  == hadc->Instance)
	{
		adc[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);

		adc[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);

		adc[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

		adc[3] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		HAL_ADCEx_InjectedStart_IT(hadc);
	}

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	xSemaphoreTake(clear_frequency_semaphore_handle, portMAX_DELAY );
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

