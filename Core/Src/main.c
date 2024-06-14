/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "defaults.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#include "mb.h"
#include "mbport.h"
#include "user_mb_app.h"
#include "user_mb_handlers.h"
#include "tools.h"
#include "user_eeprom.h"
#include "user_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLINK_WAIT_PERIOD_MS	500
#define BLINK_OP_PERIOD_MS		250

#define SCHEDULE_PROC_STOPPED 0xff

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
unsigned short DeviceType = 5;
unsigned char SN[12];
unsigned short SlaveID = MB_SlaveID;
unsigned short LastCommand = 0;

unsigned short* MB3Regs[MB_MAX3regs];
unsigned short* MB4Regs[MB_MAX4regs];

volatile unsigned short ADC_Data[6];


float k1 = DEF_k1;
float b1 = DEF_b1;
float k2 = DEF_k2;
float b2 = DEF_b2;
float k3 = DEF_k3;
float b3 = DEF_b3;
float k4 = DEF_k4;
float b4 = DEF_b4;

volatile float CurrentO2 = 0.0f;
volatile float CurrentCO2 = 0.0f;
float TargetO2 = 18.0f;
float TargetCO2 = 2.0f;
volatile float CurrentPress1 = 0.0f;
volatile float CurrentPress2 = 0.0f;

unsigned short Valve1 = 50;
unsigned short Valve2 = 50;
unsigned short KoefN2 = 80;
unsigned short KoefCO2 = 80;

float Volume = Tank_V;
float SValve_N2 = S_N2;
float SValve_CO2 = S_CO2;

unsigned short OperationState = 0;
EEPROM_Statuses_t UseEEPROM = EEPROM_FREE;

PlanElement_t Plan[MAX_Plan];
unsigned short CurrentPlan = SCHEDULE_PROC_STOPPED;
unsigned short PlanSize = 0;

static uint32_t msBlinkCounter = 0;
static uint32_t MsScheduleUpdateCounter = PLAN_Update_Time * 1000;
static uint32_t msBlinkPeriod = BLINK_WAIT_PERIOD_MS;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Assign_MB_Mem(void);
void FillDeviceSN(void);


/*
void OpenN2(void);
void OpenCO2(void);
void CloseN2(void);
void CloseCO2(void);

int CalcN2(int PlanIndex); //return milliseconds
int CalcCO2(int PlanIndex);
void CreatePlan(void);
*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		HAL_UART_AbortTransmit_IT(&huart2);
		HAL_UART_AbortReceive_IT(&huart2);
//		UART2_RX_Pos = 0;
	//	UART2_RX_Buf[0] = 0;
//		HAL_UART_Receive_IT(&huart2, &UART2_RX_Char, 1);
	}
	if (huart == &huart3)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_UART_AbortTransmit_IT(&huart3);
		HAL_UART_AbortReceive_IT(&huart3);
	//	UART3_RX_Pos = 0;
//		UART3_RX_Buf[0] = 0;
//		HAL_UART_Receive_IT(&huart3, &UART3_RX_Char, 1);
	}
}

void WriteUART2(void)
{

}

void WriteUART3(void)
{
	int k = 0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	for (int i =0; i < 100; i++) k++;
	//HAL_Delay(10);
//	HAL_UART_Transmit_IT(&huart3, (unsigned char*)UART3_TX_Buf, UART3_TX_Pos);
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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	Assign_MB_Mem();

	InitEEPROM(&hi2c1);
	// Восстановление параметров из памяти
	UseEEPROM = LoadFromEEPROM();

	htim1.Instance->CCR1 = 30 * Valve1; //reset - may change on load
	htim2.Instance->CCR1 = 30 * Valve2;

	FillDeviceSN();
	InitRelays(&htim2, &htim1);
	
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Data, 6);
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);
	
	HAL_NVIC_EnableIRQ(TIM3_IRQn); //30-10-23
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	
	// ModBus Init
	eMBInit( MB_RTU, MB_SlaveID, &huart3, 115200, &htim3 );
	eMBEnable();

	// 2nd Modbus Channel
	userMBInit(MB_SlaveID, &huart2, &htim3 );
	userMBEnable();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		UpdateMesure();
		eMBPoll();
		userMBPoll();

		if (msTimer_GetFrom(msBlinkCounter) >= msBlinkPeriod) {
			msBlinkCounter = msTimer_GetStamp();

			HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
		}

		// Save settings request
		if(UseEEPROM == EEPROM_SAVE_RQST)
			UseEEPROM = SaveToEEPROM();
		
		// Operate command request
		if (LastCommand == CMD_Start) {
			if ((OperationState & SOST_PlanReady) && !(OperationState & SOST_Running))
			{
				OperationState |= SOST_Running;
				CurrentPlan = 0;
			}
		}
		else if (LastCommand == CMD_Stop) {
			OperationState &= ~(SOST_OpenCO2 | SOST_OpenN2 | SOST_Running | SOST_PlanStarted);
			CurrentPlan = SCHEDULE_PROC_STOPPED;
		}

		// Если план готов и пришла команда на начало работы
		if (OperationState & SOST_Running) {
			ProcSchedule();
			msBlinkPeriod = BLINK_OP_PERIOD_MS;
		}
		else  //Not running = watching
		{
			CloseCO2();
			CloseN2();
			msBlinkPeriod = BLINK_WAIT_PERIOD_MS;

			//обновлять план, если просто наблюдаем каждые PLAN_Update_Time секунд.
			if (msTimer_GetFrom(MsScheduleUpdateCounter) > PLAN_Update_Time * 1000) {
				MsScheduleUpdateCounter = msTimer_GetStamp();
				OperationState |= CreateSchedule();
			}
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
		//TIM1 - PWM азота
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 24;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3000;
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
  sConfigOC.Pulse = 1500;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
		//TIM2 - PWM СО2
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3000;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
	//Счетчик длительности периодов, собственный период 100 мс
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
	//Автозапуск АЦП
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
		HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
		HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS485_RTS_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : BOARD_LED_Pin */
  GPIO_InitStruct.Pin = BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_RTS_Pin PB8 PB9 */
  GPIO_InitStruct.Pin = RS485_RTS_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


	

	
void Assign_MB_Mem(void)
{
	int i;
/*	//Original
	MB3Regs[1] = &DeviceType;
	MB3Regs[2] = (unsigned short*)&SN[0];
	MB3Regs[3] = (unsigned short*)&SN[2];
	MB3Regs[4] = (unsigned short*)&SN[4];
	MB3Regs[5] = (unsigned short*)&SN[6];
	MB3Regs[6] = (unsigned short*)&SN[8];
	MB3Regs[7] = (unsigned short*)&SN[10];
	MB3Regs[8] = &Sost; //reserved, for define assign to Sost
	MB3Regs[9] = &Sost; //reserved, for define assign to Sost
	MB3Regs[10] = (unsigned short*)&CurrentO2;
	MB3Regs[11] = MB3Regs[10]++;
	MB3Regs[12] = (unsigned short*)&CurrentCO2;
	MB3Regs[13] = MB3Regs[12]++;
	MB3Regs[14] = (unsigned short*)&CurrentPress1;
	MB3Regs[15] = MB3Regs[14]++;
	MB3Regs[16] = (unsigned short*)&CurrentPress2;
	MB3Regs[17] = MB3Regs[16]++;
	MB3Regs[18] = &Sost;
	MB3Regs[19] = (unsigned short*)&ADC_Data[0];
	MB3Regs[20] = (unsigned short*)&ADC_Data[1];
	MB3Regs[21] = (unsigned short*)&ADC_Data[2];
	MB3Regs[22] = (unsigned short*)&ADC_Data[3];
	MB3Regs[23] = &PlanSize;

for (i = 0; i < MAX_Plan; i++)
	{
		MB3Regs[24 + 5*i] = (unsigned short*)&Plan[i].Value;
		MB3Regs[25 + 5*i] = MB3Regs[23 + 5*i]++;
		MB3Regs[26 + 5*i] = (unsigned short*)&Plan[i].TimeMs;
		MB3Regs[27 + 5*i] = MB3Regs[25 + 5*i]++;
		MB3Regs[28 + 5*i] = (unsigned short*)&Plan[i].GasType;
	}

	MB4Regs[1] = &SlaveID;
	MB4Regs[2] = (unsigned short*)&TargetO2;
	MB4Regs[3] = MB4Regs[2]++;
	MB4Regs[4] = (unsigned short*)&TargetCO2;
	MB4Regs[5] = MB4Regs[4]++;
	MB4Regs[6] = &Valve1;
	MB4Regs[7] = &Valve2;
	MB4Regs[8] = &KoefN2;
	MB4Regs[9] = &KoefCO2;
	MB4Regs[10] = (unsigned short*)&Volume;
	MB4Regs[11] = MB4Regs[10]++;
	MB4Regs[12] = &LastCommand;
	MB4Regs[13] = (unsigned short*)&k1;
	MB4Regs[14] = MB4Regs[13]++;
	MB4Regs[15] = (unsigned short*)&b1;
	MB4Regs[16] = MB4Regs[15]++;
	MB4Regs[17] = (unsigned short*)&k2;
	MB4Regs[18] = MB4Regs[17]++;
	MB4Regs[19] = (unsigned short*)&b2;
	MB4Regs[20] = MB4Regs[19]++;
	MB4Regs[21] = (unsigned short*)&k3;
	MB4Regs[22] = MB4Regs[21]++;
	MB4Regs[23] = (unsigned short*)&b3;
	MB4Regs[24] = MB4Regs[23]++;
	MB4Regs[25] = (unsigned short*)&k4;
	MB4Regs[26] = MB4Regs[25]++;
	MB4Regs[27] = (unsigned short*)&b4;
	MB4Regs[28] = MB4Regs[27]++;
	*/
	//reverce word order for float
	MB3Regs[1] = &DeviceType;
	MB3Regs[2] = (unsigned short*)&SN[0];
	MB3Regs[3] = (unsigned short*)&SN[2];
	MB3Regs[4] = (unsigned short*)&SN[4];
	MB3Regs[5] = (unsigned short*)&SN[6];
	MB3Regs[6] = (unsigned short*)&SN[8];
	MB3Regs[7] = (unsigned short*)&SN[10];
	MB3Regs[8] = &OperationState; //reserved, for define assign to Sost
	MB3Regs[9] = &OperationState; //reserved, for define assign to Sost
	MB3Regs[11] = (unsigned short*)&CurrentO2;
	MB3Regs[10] = MB3Regs[11]++;
	MB3Regs[13] = (unsigned short*)&CurrentCO2;
	MB3Regs[12] = MB3Regs[13]++;
	MB3Regs[15] = (unsigned short*)&CurrentPress1;
	MB3Regs[14] = MB3Regs[15]++;
	MB3Regs[17] = (unsigned short*)&CurrentPress2;
	MB3Regs[16] = MB3Regs[17]++;
	MB3Regs[18] = &OperationState;
	MB3Regs[19] = (unsigned short*)&ADC_Data[0];
	MB3Regs[20] = (unsigned short*)&ADC_Data[1];
	MB3Regs[21] = (unsigned short*)&ADC_Data[2];
	MB3Regs[22] = (unsigned short*)&ADC_Data[3];
	MB3Regs[23] = &PlanSize;
	MB3Regs[24] = &CurrentPlan;

	for (i = 0; i < MAX_Plan; i++)
	{
		MB3Regs[26 + 5*i] = (unsigned short*)&Plan[i].Value;
		MB3Regs[25 + 5*i] = MB3Regs[26 + 5*i]++;
		MB3Regs[28 + 5*i] = (unsigned short*)&Plan[i].TimeMs;
		MB3Regs[27 + 5*i] = MB3Regs[28 + 5*i]++;
		MB3Regs[29 + 5*i] = (unsigned short*)&Plan[i].GasType;
	}
	
	MB4Regs[1] = &SlaveID;
	MB4Regs[3] = (unsigned short*)&TargetO2;
	MB4Regs[2] = MB4Regs[3]++;
	MB4Regs[5] = (unsigned short*)&TargetCO2;
	MB4Regs[4] = MB4Regs[5]++;
	MB4Regs[6] = &Valve1;
	MB4Regs[7] = &Valve2;
	MB4Regs[8] = &KoefN2;
	MB4Regs[9] = &KoefCO2;
	MB4Regs[11] = (unsigned short*)&Volume;
	MB4Regs[10] = MB4Regs[11]++;
	MB4Regs[12] = &LastCommand;
	MB4Regs[14] = (unsigned short*)&k1;
	MB4Regs[13] = MB4Regs[14]++;
	MB4Regs[16] = (unsigned short*)&b1;
	MB4Regs[15] = MB4Regs[16]++;
	MB4Regs[18] = (unsigned short*)&k2;
	MB4Regs[17] = MB4Regs[18]++;
	MB4Regs[20] = (unsigned short*)&b2;
	MB4Regs[19] = MB4Regs[20]++;
	MB4Regs[22] = (unsigned short*)&k3;
	MB4Regs[21] = MB4Regs[22]++;
	MB4Regs[24] = (unsigned short*)&b3;
	MB4Regs[23] = MB4Regs[24]++;
	MB4Regs[26] = (unsigned short*)&k4;
	MB4Regs[25] = MB4Regs[26]++;
	MB4Regs[28] = (unsigned short*)&b4;
	MB4Regs[27] = MB4Regs[28]++;
	MB4Regs[29] = &UseEEPROM;
}

void FillDeviceSN(void)
{
	unsigned short *u, v;
	unsigned int i, *p;

	u = (unsigned short*)(UID_BASE);
	v = *u;
	SN[0] = (v >> 8) & 0xFF;
	SN[1] = v & 0xFF;
  u = (unsigned short*)(UID_BASE + 0x02);
	v = *u;
	SN[2] = (v >> 8) & 0xFF;
	SN[3] = v & 0xFF;
	p = (unsigned int*)(UID_BASE + 0x04);
	i = *p;
	SN[4] = (i >> 24) & 0xFF;
	SN[5] = (i >> 16) & 0xFF;
	SN[6] = (i >> 8) & 0xFF;
	SN[7] = i & 0xFF;
	p = (unsigned int*)(UID_BASE + 0x08);
	i = *p;
	SN[8] = (i >> 24) & 0xFF;
	SN[9] = (i >> 16) & 0xFF;
	SN[10] = (i >> 8) & 0xFF;
	SN[11] = i & 0xFF;
}


uint16_t mb_user_data_get(MB_User_Types_t type, uint16_t index) {

	static uint16_t index_prev;

	index++;

	switch(type) {
	case MB_T_IN_REG: {

		if(index >= 10 && index <= 17) {
			if(index == index_prev + 1) {
				float value = *((float*)MB3Regs[index_prev]);
				int32_t d = value * 65536.0f;
				index_prev = 0;
				return (d >> 16);

			} else {
				float value = *((float*)MB3Regs[index]);
				int32_t d = value * 65536.0f;
				index_prev = index;

				return d & 0xFFFF;
			}
		}
		return *MB3Regs[index];

	}

	case MB_T_HOLD_REG: {
		if((index >= 2 && index <= 5) || (index == 10) || (index == 11) ||
							(index >= 13 && index <= 28)) {
			if(index == index_prev + 1) {
				float value = *((float*)MB4Regs[index_prev]);
				int32_t d = value * 65536.0f;
				index_prev = 0;
				return (d >> 16);

			} else {
				float value = *((float*)MB4Regs[index]);
				int32_t d = value * 65536.0f;
				index_prev = index;

				return d & 0xFFFF;
			}
		}
		else
			return *MB4Regs[index];
	}

	case MB_T_COIL:
		return UINT16_MAX;
	case MB_T_DIN:
		return UINT16_MAX;

	default:
		return UINT16_MAX;
	}

}

uint16_t mb_user_data_set(MB_User_Types_t type, uint16_t index, uint16_t data) {

	static FourByte_t FB;
	static uint16_t index_prev;
	// increment index because of MB4Regs and MB3Regs offset by 1
	index++;

	switch(type) {

	case MB_T_HOLD_REG:
		uint16_t T = data;
		//
		if(index >= 6 && index <= 9) {
			if (!(OperationState & SOST_Running)) {
				if (T <= 100) {
					// update Valve1 or Valve2
					*MB4Regs[index] = T;

					htim1.Instance->CCR1 = 30 * Valve1; //reset on all change in (6-9) - for small code
					htim2.Instance->CCR1 = 30 * Valve2;

					OperationState &= ~(SOST_PlanReady | SOST_PlanStarted);
				}
			}
		}
		else if(index == 12) {
			if ((T == CMD_Start) || (T == CMD_Stop))
				LastCommand = T;

		}
		else {

			// 4 bytes parameters len
			if((index >= 2 && index <= 5) || (index == 10) || (index == 11) ||
					(index >= 13 && index <= 28)) {

				if(index == index_prev + 1) {
					FB.w[1] = data;
					float *value = (float*)MB4Regs[index_prev];
					*value = FB.i * 0.0000152587890625f; // / 65536.0f;

					index_prev = 0;

				} else {
					FB.w[0] = data;
					index_prev = index;
				}
			}
			// 2 bytes parameters len
			else
				*MB4Regs[index] = data;

		}

		break;

	case MB_T_COIL:
		break;

	default:
		return UINT16_MAX;
	}
	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart3.Instance)
	{
		xMBPortSerialRxCpltCallback(huart);
	}
	else if(huart->Instance == huart2.Instance) {
		userMBRxCallBack(huart);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart3.Instance)
	{
		xMBPortSerialTxCpltCallback(huart);
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
