/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Gas_Pressure_Pin GPIO_PIN_1
#define Gas_Pressure_GPIO_Port GPIOB
#define Float_Chamber1_min_Pin GPIO_PIN_12
#define Float_Chamber1_min_GPIO_Port GPIOB
#define Float_Chamber1_max_Pin GPIO_PIN_13
#define Float_Chamber1_max_GPIO_Port GPIOB
#define Float_Chamber2_min_Pin GPIO_PIN_14
#define Float_Chamber2_min_GPIO_Port GPIOB
#define Float_Chamber2_max_Pin GPIO_PIN_15
#define Float_Chamber2_max_GPIO_Port GPIOB
#define Float_Chamber3_min_Pin GPIO_PIN_8
#define Float_Chamber3_min_GPIO_Port GPIOA
#define Float_Chamber3_max_Pin GPIO_PIN_9
#define Float_Chamber3_max_GPIO_Port GPIOA
#define Float_Chamber4_min_Pin GPIO_PIN_10
#define Float_Chamber4_min_GPIO_Port GPIOA
#define Float_Chamber4_max_Pin GPIO_PIN_11
#define Float_Chamber4_max_GPIO_Port GPIOA
#define Electrode1_Output_Pin GPIO_PIN_12
#define Electrode1_Output_GPIO_Port GPIOA
#define Electrode2_Output_Pin GPIO_PIN_15
#define Electrode2_Output_GPIO_Port GPIOA
#define Electrode3_Output_Pin GPIO_PIN_3
#define Electrode3_Output_GPIO_Port GPIOB
#define Electrode4_Output_Pin GPIO_PIN_4
#define Electrode4_Output_GPIO_Port GPIOB
#define Solenoid1_Output_Pin GPIO_PIN_5
#define Solenoid1_Output_GPIO_Port GPIOB
#define Solenoid2_Output_Pin GPIO_PIN_6
#define Solenoid2_Output_GPIO_Port GPIOB
#define Solenoid3_Output_Pin GPIO_PIN_7
#define Solenoid3_Output_GPIO_Port GPIOB
#define Solenoid4_Output_Pin GPIO_PIN_8
#define Solenoid4_Output_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern int *electrode_power_status;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
