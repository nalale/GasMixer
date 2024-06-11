/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "defaults.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	MB_T_IN_REG,
	MB_T_HOLD_REG,
	MB_T_COIL,
	MB_T_DIN,
} MB_User_Types_t;

typedef enum {
	GasType_None = 0,
	GasType_N2 = 1,
	GasType_O2 = 2,
} GasType_t;

typedef struct PlanElement {
	float Value;
	uint32_t TimeMs;
	short GasType;
} PlanElement_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern float k1;
extern float b1;
extern float k2;
extern float b2;
extern float k3;
extern float b3;
extern float k4;
extern float b4;

extern float TargetO2;
extern float TargetCO2;

extern unsigned short Valve1;
extern unsigned short Valve2;
extern unsigned short KoefN2;
extern unsigned short KoefCO2;

extern float Volume;
extern float SValve_N2;
extern float SValve_CO2;

extern unsigned short OperationState;
extern PlanElement_t Plan[];
extern unsigned short CurrentPlan;
extern unsigned short PlanSize;
extern uint32_t Ms10Counter;

extern volatile float CurrentO2;
extern volatile float CurrentCO2;
extern volatile float CurrentPress1;
extern volatile float CurrentPress2;
extern volatile unsigned short ADC_Data[];

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

uint16_t mb_user_data_get(MB_User_Types_t type, uint16_t index);
uint16_t mb_user_data_set(MB_User_Types_t type, uint16_t index, uint16_t data);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOARD_LED_Pin GPIO_PIN_13
#define BOARD_LED_GPIO_Port GPIOC
#define RS485_RTS_Pin GPIO_PIN_14
#define RS485_RTS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define USER_REG_INPUTS_START	0
#define USER_REG_INPUTS_NUM		MB_MAX3regs // //7

#define USER_REG_HOLDING_START	0
#define USER_REG_HOLDING_NUM	MB_MAX4regs //2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
