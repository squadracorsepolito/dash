/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f3xx_hal.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_gpio.h"

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
#define EBS_RELAY1_Pin GPIO_PIN_10
#define EBS_RELAY1_GPIO_Port GPIOD
#define EBS_RELAY2_Pin GPIO_PIN_11
#define EBS_RELAY2_GPIO_Port GPIOD
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
#define PWM3_CMD_Pin GPIO_PIN_10
#define PWM3_CMD_GPIO_Port GPIOC
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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
