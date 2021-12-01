/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_1
#define LED_B_GPIO_Port GPIOF
#define SPARE_IN_Pin GPIO_PIN_2
#define SPARE_IN_GPIO_Port GPIOC
#define RTD_BUTTON_Pin GPIO_PIN_0
#define RTD_BUTTON_GPIO_Port GPIOA
#define BMS_LED_Pin GPIO_PIN_10
#define BMS_LED_GPIO_Port GPIOE
#define NOHV_LED_Pin GPIO_PIN_12
#define NOHV_LED_GPIO_Port GPIOE
#define IMD_LED_Pin GPIO_PIN_14
#define IMD_LED_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOE
#define RTD_LED_Pin GPIO_PIN_10
#define RTD_LED_GPIO_Port GPIOB
#define PWM_PUMP_Pin GPIO_PIN_13
#define PWM_PUMP_GPIO_Port GPIOD
#define PWM_RAD_FAN_Pin GPIO_PIN_14
#define PWM_RAD_FAN_GPIO_Port GPIOD
#define BP_FAN_Pin GPIO_PIN_15
#define BP_FAN_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
