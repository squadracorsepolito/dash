/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.h
 * @brief   This file contains all the function prototypes for
 *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

    /* USER CODE BEGIN Includes */
    /* USER CODE END Includes */

    extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
#define CHANGEME 0x69

#define BOOTLOADER_ID_CAN 0x004
#define PING_ID_CAN 0x146 // TODO: remove
#define TLB_ERROR_ID_CAN 0x020
#define AS_STATE_ID_CAN CHANGEME
#define MISSION_STATUS_ID_CAN (CHANGEME + 1)
#define CMD_EBS_ID_CAN (CHANGEME + 2)
#define ACK_RTD_ID_CAN 0x040
#define PWM_ID_CAN 0x041
#define CMD_RTD_ID_CAN 0x042
#define CMD_COUNTER_REQUEST_ID_CAN 0x043
#define COUNTER_REPLY_ID_CAN 0x044
#define SENSORBOARD_4_7_ID_CAN 0x11

    /* USER CODE END Private defines */

    void MX_CAN_Init(void);

    /* USER CODE BEGIN Prototypes */
    void CAN_Msg_Send(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox, uint32_t TimeOut);

    /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
