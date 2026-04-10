/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M1_TIM2_CH1_Pin GPIO_PIN_0
#define M1_TIM2_CH1_GPIO_Port GPIOA
#define M2_TIM2_CH2_Pin GPIO_PIN_1
#define M2_TIM2_CH2_GPIO_Port GPIOA
#define D_TX_Pin GPIO_PIN_2
#define D_TX_GPIO_Port GPIOA
#define D_RX_Pin GPIO_PIN_3
#define D_RX_GPIO_Port GPIOA
#define S1_TIM3_CH3_Pin GPIO_PIN_0
#define S1_TIM3_CH3_GPIO_Port GPIOB
#define S2_TIM3_CH4_Pin GPIO_PIN_1
#define S2_TIM3_CH4_GPIO_Port GPIOB
#define JY901_SCL_Pin GPIO_PIN_10
#define JY901_SCL_GPIO_Port GPIOB
#define JY901_SDA_Pin GPIO_PIN_11
#define JY901_SDA_GPIO_Port GPIOB
#define MT_TX_Pin GPIO_PIN_9
#define MT_TX_GPIO_Port GPIOA
#define MT_RX_Pin GPIO_PIN_10
#define MT_RX_GPIO_Port GPIOA
#define M1_DIR_Pin GPIO_PIN_12
#define M1_DIR_GPIO_Port GPIOA
#define M2_DIR_Pin GPIO_PIN_15
#define M2_DIR_GPIO_Port GPIOA
#define M3_DIR_Pin GPIO_PIN_3
#define M3_DIR_GPIO_Port GPIOB
#define M4_DIR_Pin GPIO_PIN_4
#define M4_DIR_GPIO_Port GPIOB
#define M5_DIR_Pin GPIO_PIN_5
#define M5_DIR_GPIO_Port GPIOB
#define M3_TIM4_CH1_Pin GPIO_PIN_6
#define M3_TIM4_CH1_GPIO_Port GPIOB
#define M4_TIM4_CH2_Pin GPIO_PIN_7
#define M4_TIM4_CH2_GPIO_Port GPIOB
#define M5_TIM4_CH3_Pin GPIO_PIN_8
#define M5_TIM4_CH3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
