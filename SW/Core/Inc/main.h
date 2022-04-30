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
#define EBS_RELAY1_CMD_Pin GPIO_PIN_10
#define EBS_RELAY1_CMD_GPIO_Port GPIOD
#define EBS_RELAY2_CMD_Pin GPIO_PIN_11
#define EBS_RELAY2_CMD_GPIO_Port GPIOD
#define TS_CK_STM_Pin GPIO_PIN_12
#define TS_CK_STM_GPIO_Port GPIOD
#define SpareButton_STM_Pin GPIO_PIN_13
#define SpareButton_STM_GPIO_Port GPIOD
#define BUZZERAS_CMD_Pin GPIO_PIN_14
#define BUZZERAS_CMD_GPIO_Port GPIOD
#define AMS_CMD_Pin GPIO_PIN_6
#define AMS_CMD_GPIO_Port GPIOC
#define BUZZEREV_CMD_Pin GPIO_PIN_7
#define BUZZEREV_CMD_GPIO_Port GPIOC
#define IMD_CMD_Pin GPIO_PIN_8
#define IMD_CMD_GPIO_Port GPIOC
#define TS_EX_STM_Pin GPIO_PIN_9
#define TS_EX_STM_GPIO_Port GPIOC
#define ASB_CMD_Pin GPIO_PIN_8
#define ASB_CMD_GPIO_Port GPIOA
#define TSOFF_CMD_Pin GPIO_PIN_11
#define TSOFF_CMD_GPIO_Port GPIOC
#define RTD_CMD_Pin GPIO_PIN_12
#define RTD_CMD_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOD
#define AMI3_CMD_Pin GPIO_PIN_4
#define AMI3_CMD_GPIO_Port GPIOD
#define AMI2_CMD_Pin GPIO_PIN_6
#define AMI2_CMD_GPIO_Port GPIOD
#define AMI1_CMD_Pin GPIO_PIN_5
#define AMI1_CMD_GPIO_Port GPIOB
#define ASSI_YELLOW_CMD_Pin GPIO_PIN_7
#define ASSI_YELLOW_CMD_GPIO_Port GPIOB
#define ASSI_BLUE_CMD_Pin GPIO_PIN_8
#define ASSI_BLUE_CMD_GPIO_Port GPIOB
#define PWM1_CMD_Pin GPIO_PIN_0
#define PWM1_CMD_GPIO_Port GPIOE
#define PWM2_CMD_Pin GPIO_PIN_1
#define PWM2_CMD_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define PWM_PUMP_TIM htim16
#define PWM_PUMP_CH TIM_CHANNEL_1
#define PWM_RAD_TIM htim17
#define PWM_RAD_CH TIM_CHANNEL_1
#define PWM_BP_TIM htim8
#define PWM_BP_CH TIM_CHANNEL_1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
