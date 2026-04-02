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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM3autoreload TIMxfrequency/((TIM3prescaler+1)*TIM3frequency)
#define TIM3frequency 1000
#define TIM6frequency 1000
#define TIM6autoreload TIMxfrequency/((TIM6prescaler+1)*TIM6frequency)
#define TIM3prescaler 0
#define TIMxfrequency 16000000
#define TIM6prescaler 0
#define TIM6frequency 1000
#define TIM6autoreload TIMxfrequency/((Tim6prescaler+1)*Tim6frequency)
#define SENSE_A_Pin GPIO_PIN_0
#define SENSE_A_GPIO_Port GPIOA
#define SENSE_B_Pin GPIO_PIN_1
#define SENSE_B_GPIO_Port GPIOA
#define M2_Pin GPIO_PIN_0
#define M2_GPIO_Port GPIOB
#define M1_Pin GPIO_PIN_1
#define M1_GPIO_Port GPIOB
#define Solenoid_CTRL2_Pin GPIO_PIN_7
#define Solenoid_CTRL2_GPIO_Port GPIOC
#define Solenoid_CTRL3_Pin GPIO_PIN_8
#define Solenoid_CTRL3_GPIO_Port GPIOC
#define Solenoid_CTRL4_Pin GPIO_PIN_9
#define Solenoid_CTRL4_GPIO_Port GPIOC
#define Solenoid_CTRL1_Pin GPIO_PIN_8
#define Solenoid_CTRL1_GPIO_Port GPIOA
#define Home_Button_Pin GPIO_PIN_11
#define Home_Button_GPIO_Port GPIOA
#define Home_SENS_Pin GPIO_PIN_12
#define Home_SENS_GPIO_Port GPIOA
#define M_SWDIO_Pin GPIO_PIN_13
#define M_SWDIO_GPIO_Port GPIOA
#define M_SWCLK_Pin GPIO_PIN_14
#define M_SWCLK_GPIO_Port GPIOA
#define Solenoid_CTRL5_Pin GPIO_PIN_10
#define Solenoid_CTRL5_GPIO_Port GPIOC
#define M_SWO_Pin GPIO_PIN_3
#define M_SWO_GPIO_Port GPIOB
#define DEBUG_Pin GPIO_PIN_9
#define DEBUG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
