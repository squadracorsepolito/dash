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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTD_BTN_Pin GPIO_PIN_6
#define RTD_BTN_GPIO_Port GPIOA
#define AMS_ERR_CMD_Pin GPIO_PIN_0
#define AMS_ERR_CMD_GPIO_Port GPIOB
#define BUZZEREV_CMD_Pin GPIO_PIN_1
#define BUZZEREV_CMD_GPIO_Port GPIOB
#define IMD_ERR_CMD_Pin GPIO_PIN_2
#define IMD_ERR_CMD_GPIO_Port GPIOB
#define EXT_BTN_Pin GPIO_PIN_10
#define EXT_BTN_GPIO_Port GPIOB
#define SD_CLOSED_CMD_Pin GPIO_PIN_11
#define SD_CLOSED_CMD_GPIO_Port GPIOB
#define TSOFF_CMD_Pin GPIO_PIN_13
#define TSOFF_CMD_GPIO_Port GPIOB
#define RTD_CMD_Pin GPIO_PIN_14
#define RTD_CMD_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOA
#define SD_CMD_Pin GPIO_PIN_6
#define SD_CMD_GPIO_Port GPIOB
#define BAT_FAN_PWM_CMD_Pin GPIO_PIN_8
#define BAT_FAN_PWM_CMD_GPIO_Port GPIOB
#define RADIATOR_FANS_PWM_CMD_Pin GPIO_PIN_9
#define RADIATOR_FANS_PWM_CMD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define PCBVER 2

// PCB ver 1 (Francesco Minichelli's original design)
#if PCBVER == 1

#define BAT_FAN_PWM_TIM htim16
#define BAT_FAN_PWM_CH TIM_CHANNEL_1

#define POWERTRAIN_COOLING_PWM_TIM htim17
#define POWERTRAIN_COOLING_PWM_CH TIM_CHANNEL_1

#define ASB_MOTOR_PWM_TIM htim8
#define ASB_MOTOR_PWM_CH TIM_CHANNEL_1

#define COUNTER_TIM htim3

// PCB ver 2 (the black one)
#elif PCBVER == 2

#define BAT_FAN_PWM_TIM htim16
#define BAT_FAN_PWM_CH TIM_CHANNEL_1

#define RADIATOR_FANS_PWM_TIM htim17
#define RADIATOR_FANS_PWM_CH TIM_CHANNEL_1

#define COUNTER_TIM htim2

#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
