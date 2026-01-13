/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_START_Pin GPIO_PIN_0
#define BUTTON_START_GPIO_Port GPIOA
#define BUTTON_STOP_Pin GPIO_PIN_1
#define BUTTON_STOP_GPIO_Port GPIOA
#define MOTOR1_ENA_Pin GPIO_PIN_4
#define MOTOR1_ENA_GPIO_Port GPIOA
#define MOTOR1_DIR_Pin GPIO_PIN_5
#define MOTOR1_DIR_GPIO_Port GPIOA
#define MOTOR2_ENA_Pin GPIO_PIN_6
#define MOTOR2_ENA_GPIO_Port GPIOA
#define MOTOR2_DIR_Pin GPIO_PIN_7
#define MOTOR2_DIR_GPIO_Port GPIOA
#define ENCODER_CLK_Pin GPIO_PIN_3
#define ENCODER_CLK_GPIO_Port GPIOB
#define ENCODER_DT_Pin GPIO_PIN_4
#define ENCODER_DT_GPIO_Port GPIOB
#define ENCODER_SW_Pin GPIO_PIN_5
#define ENCODER_SW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
