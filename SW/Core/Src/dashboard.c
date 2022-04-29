/*INCLUDE*/

#include "dashboard.h"
#include <stdio.h>

/*EXTERNAL GLOBAL VARIABLES*/

extern char 				msg[30];
extern char 				value[60];


extern CAN_HandleTypeDef 	hcan;

// TODO: move to tim.c
extern TIM_HandleTypeDef 	htim8;
extern TIM_HandleTypeDef 	htim16;
extern TIM_HandleTypeDef 	htim17;
extern UART_HandleTypeDef 	huart1;


extern CAN_TxHeaderTypeDef	TxHeader;
extern CAN_RxHeaderTypeDef 	RxHeader;
extern CAN_FilterTypeDef    sFilterConfig;
extern uint32_t             TxMailbox;
extern uint8_t              TxData[8];
extern uint8_t              RxData[8];
extern uint32_t 			counter;

/*Error Variables*/
extern bool IMD_ERR;
extern bool BMS_ERR;
extern bool NOHV;
extern bool ERR_BYTE_RECEIVED;

/*PWM Variables*/
extern uint8_t PWM_RAD_FAN;
extern uint8_t PWM_PUMP;
extern uint8_t PWM_BP_FAN;

/*State Machine for RTD*/
extern enum state {
    IDLE = 0,
    CTOR_EN = 1,
    WAIT_CTOR_EN_ACK = 2,
    RTD_EN = 3,
    WAIT_RTD_EN_ACK = 4,
    RTD = 5,
    STOP = 6
} rtd_fsm;


/*RTD_FSM variables*/
extern bool CTOR_EN_ACK;
extern bool RTD_EN_ACK;
extern bool REBOOT_FSM;

/*Ping variable*/
extern bool ping;

/*Front brake pressure value*/
extern uint16_t brake;

/*CUSTOM FUNCTIONS*/

/*Toggle a specific GPIO*/
void LedBlinking(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t period)
{

	static uint32_t delay_100us_last = 0;

	if(delay_fun(&delay_100us_last,period))
	{

		HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);

	}


}

/*Return the value of the counter that is incremented every 100us*/
uint32_t ReturnTime_100us(void)
{
	return counter;
}


/*Setup can header and start peripheral*/
/*
APB1 = 30 MHz
Periphelal clock = APB1/Prescaler = 30/3 = 10 Mhz

Time Quanta = SJW (start_bit) + BS1 + BS2 = 8 + 1 + 1 = 10

Periphelal Clock / Time Quanta = CAN bitrate -> 10Mhz/10 = 1Mbit/s

or prescaler 6; Clock 30/6 = 5Mhz
Time Quanta = 2 + 2 + 1
*/
void CAN_Config(void)
{
	if (HAL_CAN_Start(&hcan) != HAL_OK)
  	{
	    /* Start Error */
	    Error_Handler();
  	}
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
	    /* Start Error */
	    Error_Handler();
  	}
	/*Enable interrupt*/


	
}

/*Rx Message interrupt from CAN*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
		/* Transmission request Error */
		//HAL_CAN_ResetError(hcan);
		//Error_Handler();
    }

    /*Reboot Board - Received command byte from CAN*/
	else if ((RxHeader.StdId == BOOTLOADER_ID_CAN) && (RxHeader.DLC == 2) && (RxData[0] == 0xFF) && (RxData[1] == 0x00))
	{
     	NVIC_SystemReset();
     	
	}
	/*Ping command received*/
	else if ((RxHeader.StdId == PING_ID_CAN) && (RxHeader.DLC == 2) && (RxData[0] == 0x00) && (RxData[1] == 0xFF))
		{
	     	ping = true;

		}
	/*Received TLB error byte in order to turn LEDs on or off*/
	else if ((RxHeader.StdId == TLB_ERROR_ID_CAN) && (RxHeader.DLC == 1) && (RxData[0] < 16))
	{
     	NOHV = (bool)(RxData[0] & 1);
     	BMS_ERR = (bool)(RxData[0] & 4) || (bool)(RxData[0] & 2);
     	IMD_ERR = (bool)(RxData[0] & 4);
     	ERR_BYTE_RECEIVED = true;
     	
	}
	/*Ready to drive ACK from DSPACE*/
	else if ((RxHeader.StdId == ACK_RTD_ID_CAN) && (RxHeader.DLC == 1))
	{
		if((RxData[0] == 1) && (rtd_fsm == WAIT_CTOR_EN_ACK))
		{
			CTOR_EN_ACK = true;
		}
		else if((RxData[0] == 2) && (rtd_fsm == WAIT_RTD_EN_ACK))
		{
			RTD_EN_ACK = true;
		}
		else if((RxData[0] == 3))
		{
			REBOOT_FSM = true;
		}	
     	
	}
	/*Set duty cycle*/
	else if ((RxHeader.StdId == PWM_ID_CAN) && (RxHeader.DLC == 3))
	{

		if(RxData[0] <= 100)
		{
			PWM_PUMP = RxData[0];
			__HAL_TIM_SET_COMPARE(&PWM_PUMP_TIM, PWM_PUMP_CH, PWM_PUMP);
		}
		if(RxData[1] <= 100)
		{
			PWM_RAD_FAN = RxData[1];
			__HAL_TIM_SET_COMPARE(&PWM_RAD_TIM, PWM_RAD_CH, PWM_RAD_FAN);
		}
		if(RxData[2] <= 100)
		{
			PWM_BP_FAN = RxData[2];
			__HAL_TIM_SET_COMPARE(&PWM_BP_TIM, PWM_BP_CH, PWM_BP_FAN);
		}

	}
	else if((RxHeader.StdId == SENSORBOARD_4_7_ID_CAN) && (RxHeader.DLC == 8))
	{
		brake = ((RxData[6] << 8) | RxData[7]);
	}

}

/*FSM*/
void ReadyToDriveFSM(uint32_t delay_100us)
{
	static uint32_t delay_100us_last = 0;
	static int counter_CTOR_EN = 0;

	if(delay_fun(&delay_100us_last,delay_100us))
	{
		static int counter_buzzer = 0;

		switch (rtd_fsm)
		{
			case IDLE :
				if(HAL_GPIO_ReadPin(TS_CK_STM_GPIO_Port, TS_CK_STM_Pin))
				{
					rtd_fsm = CTOR_EN;
				}
				else
				{
					TxHeader.StdId = CMD_RTD_ID_CAN;
					TxHeader.ExtId = CMD_RTD_ID_CAN;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.IDE = CAN_ID_STD;
					TxHeader.DLC = 1;
					TxHeader.TransmitGlobalTime = DISABLE;
					TxData[0]= 0x0;
					CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
				}
			break;

			case CTOR_EN :
				
					TxHeader.StdId = CMD_RTD_ID_CAN;
					TxHeader.ExtId = CMD_RTD_ID_CAN;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.IDE = CAN_ID_STD;
					TxHeader.DLC = 1;
					TxHeader.TransmitGlobalTime = DISABLE;
					TxData[0]= 0x1;
					CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);

					rtd_fsm = WAIT_CTOR_EN_ACK;
					
				
			break;

			case WAIT_CTOR_EN_ACK :
				
				if(CTOR_EN_ACK && REBOOT_FSM == false)
				{
					CTOR_EN_ACK = false;
					rtd_fsm = RTD_EN;
				}
				else if(REBOOT_FSM)
				{
					REBOOT_FSM = false;
					HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, OFF);
					rtd_fsm = IDLE;
				}
				else
				{
					TxHeader.StdId = CMD_RTD_ID_CAN;
					TxHeader.ExtId = CMD_RTD_ID_CAN;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.IDE = CAN_ID_STD;
					TxHeader.DLC = 1;
					TxHeader.TransmitGlobalTime = DISABLE;
					TxData[0]= 0x1;
					CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
				}
			break;

			case RTD_EN :
				if(HAL_GPIO_ReadPin(TS_CK_STM_GPIO_Port, TS_CK_STM_Pin) && (brake >= BRAKE_THRESHOLD))
				{
					
					//CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);

					rtd_fsm = WAIT_RTD_EN_ACK;
				}
				else if(REBOOT_FSM)
				{
					REBOOT_FSM = false;
					HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, OFF);
					rtd_fsm = IDLE;
				}
			break;

			case WAIT_RTD_EN_ACK :
				
				if(RTD_EN_ACK)
				{
					RTD_EN_ACK = false;
					rtd_fsm = RTD;
				}
				else if(REBOOT_FSM)
				{
					REBOOT_FSM = false;
					HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, OFF);
					rtd_fsm = IDLE;
				}
				else
				{
					TxHeader.StdId = CMD_RTD_ID_CAN;
					TxHeader.ExtId = CMD_RTD_ID_CAN;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.IDE = CAN_ID_STD;
					TxHeader.DLC = 1;
					TxHeader.TransmitGlobalTime = DISABLE;
					TxData[0]= 0x2;
					CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
				}
			break;

			case RTD :
			if(counter_buzzer < 40)
			{
				counter_buzzer++;
				HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, ON);
				HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
			}
			else
			{
				HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, OFF);
				//HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
				counter_buzzer = 0;
				rtd_fsm = STOP;
			}

			break;

			case STOP:
				if(REBOOT_FSM)
				{
					REBOOT_FSM = false;
					HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, OFF);
					rtd_fsm = IDLE;
					TxHeader.StdId = CMD_RTD_ID_CAN;
					TxHeader.ExtId = CMD_RTD_ID_CAN;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.IDE = CAN_ID_STD;
					TxHeader.DLC = 1;
					TxHeader.TransmitGlobalTime = DISABLE;
					TxData[0]= 0x0;
					CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
				}
			break;

			default:
			break;
		}

	}

}


/*Update Cockpit's LEDs*/
void UpdateCockpitLed(uint32_t delay_100us)
{
	static uint32_t delay_100us_last = 0;

	if(delay_fun(&delay_100us_last,delay_100us))
	{

		if(ERR_BYTE_RECEIVED)
		{
			ERR_BYTE_RECEIVED = false;
			HAL_GPIO_WritePin(GPIOB, AMS_CMD_Pin, BMS_ERR);
			HAL_GPIO_WritePin(GPIOE, TSOFF_CMD_Pin, NOHV);
			HAL_GPIO_WritePin(GPIOF, IMD_CMD_Pin, IMD_ERR);

			/*LED ON or OFF depending on ERR_BYTE*/

		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, AMS_CMD_Pin, ON);
			HAL_GPIO_WritePin(GPIOE, TSOFF_CMD_Pin, OFF);
			HAL_GPIO_WritePin(GPIOF, IMD_CMD_Pin, ON);
			/*LED always ON, timeout can*/
		}

	}

}



/*Setup TIMER, CAN*/
void SetupDashBoard(void)
{

	/*Start timer for PWM*/
	if (HAL_TIM_PWM_Start(&PWM_PUMP_TIM, PWM_PUMP_CH) != HAL_OK)
  	{
    /* PWM generation Error */
    	Error_Handler();
  	}

  	/*Start timer for PWM*/
	if (HAL_TIM_PWM_Start(&PWM_RAD_TIM, PWM_RAD_CH) != HAL_OK)
  	{
    /* PWM generation Error */
    	Error_Handler();
  	}

	/*Start timer for PWM*/
	if (HAL_TIM_PWM_Start(&PWM_BP_TIM, PWM_BP_CH) != HAL_OK)
	{
	/* PWM generation Error */
	    Error_Handler();
	}

	CAN_Config();

	sprintf(msg, "DashBoard Boot - v1.0\n\r");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, 30, 20);
	memset(msg,0,strlen(msg));
	sprintf(msg, "Configuration complete\n\r\n\r");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, 30, 20);

	HAL_GPIO_WritePin(GPIOB, AMS_CMD_Pin, ON);
	HAL_GPIO_WritePin(GPIOF, IMD_CMD_Pin, ON);
	HAL_GPIO_WritePin(GPIOE, TSOFF_CMD_Pin, ON);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOB, AMS_CMD_Pin, OFF);
	HAL_GPIO_WritePin(GPIOE, TSOFF_CMD_Pin, OFF);
	HAL_GPIO_WritePin(GPIOF, IMD_CMD_Pin, OFF);
	/*Proof that LEDs work*/

	#ifdef DEBUG	
	memset(msg,0,strlen(msg));
	sprintf(msg, "Debug Mode = ON\n\r\n\r");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, 30, 20);
	#endif
	
	/*BootScreen with verion of the code and message of complete configuration*/
}


/*Return true if the delay is completed, false otherwise*/
int delay_fun (uint32_t *delay_100us_last, uint32_t delay_100us)
{

	uint32_t current_time = ReturnTime_100us();

	if(current_time > *delay_100us_last && (current_time - *delay_100us_last) >= delay_100us) 
	{
		*delay_100us_last = current_time;
		return 1;
	}
	else if(current_time < *delay_100us_last && (0xFFFFFFFF - current_time - *delay_100us_last) >= delay_100us)
	{
		*delay_100us_last = current_time;
		return 1;
	}
	/*In case of timer overflow, the delay is computed correctly*/

	return 0;

}



/*Send message to CAN BUS*/
void CAN_Msg_Send(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox, uint32_t TimeOut) 
{

	static uint32_t can_counter_100us = 0;
	can_counter_100us = ReturnTime_100us(); 

	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan)<1)
	{
	
		if(delay_fun(&can_counter_100us,TimeOut)) 
		{

			//Error_Handler();
			HAL_CAN_ResetError(hcan);
			HAL_CAN_AbortTxRequest(hcan, *pTxMailbox);
		}

	}

	if (HAL_CAN_AddTxMessage(hcan, pHeader, aData, pTxMailbox) != HAL_OK)
    {
    
        /* Transmission request Error */
         // Error_Handler();
		HAL_CAN_ResetError(hcan);
		HAL_CAN_AbortTxRequest(hcan, *pTxMailbox);
    }


}

/*Send timer data to CAN BUS*/
void CAN_Tx(void)
{

	/*Ping received, reply with 0x00FF using Bootloader's ID*/
	if(ping)
	{
			ping = false;
			TxHeader.StdId = BOOTLOADER_ID_CAN;
			//TxHeader.ExtId = BOOTLOADER_ID_CAN;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.DLC = 6;
			TxHeader.TransmitGlobalTime = DISABLE;
			TxData[0] = 0x46;
			TxData[1] = rtd_fsm;
			TxData[2] = (HAL_GPIO_ReadPin(GPIOB, AMS_CMD_Pin)) | (HAL_GPIO_ReadPin(GPIOE, TSOFF_CMD_Pin)<<1) | (HAL_GPIO_ReadPin(GPIOF, IMD_CMD_Pin)<<2);
			TxData[3] = PWM_PUMP;
			TxData[4] = PWM_RAD_FAN;
			TxData[5] = PWM_BP_FAN;
			CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);

	}


}


/*Print on uart */
void Debug_UART(int state)
{
	

}

void Debug_CAN_Tx(uint32_t delay_100us)
{
	static uint32_t delay_100us_last = 0;

	if(delay_fun(&delay_100us_last,delay_100us))
	{
		TxHeader.StdId = DASH_STATUS;
		//TxHeader.ExtId = BOOTLOADER_ID_CAN;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.DLC = 6;
		TxHeader.TransmitGlobalTime = DISABLE;
		TxData[0] = 0x46;
		TxData[1] = rtd_fsm;
		TxData[2] = (HAL_GPIO_ReadPin(GPIOB, AMS_CMD_Pin)) | (HAL_GPIO_ReadPin(GPIOE, TSOFF_CMD_Pin)<<1) | (HAL_GPIO_ReadPin(GPIOF, IMD_CMD_Pin)<<2);
		TxData[3] = PWM_PUMP;
		TxData[4] = PWM_RAD_FAN;
		TxData[5] = PWM_BP_FAN;
		CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
	}


}

/*Core SensorBoard*/
void CoreDashBoard(void)
{

	LedBlinking(LED2_GPIO_Port, LED2_Pin , 1000);
	//Blink green led

	UpdateCockpitLed(5000);
	/*Update state Cockpit's LEDs*/

	ReadyToDriveFSM(500);
	/*Ready to drive FSM*/

	CAN_Tx();
	/*Send timer data via CAN*/

	Debug_CAN_Tx(500);
	/*Send debug packet*/
	

}
