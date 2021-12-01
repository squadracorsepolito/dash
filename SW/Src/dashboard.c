/*INCLUDE*/

#include "dashboard.h"

/*EXTERNAL GLOBAL VARIABLES*/

extern char 				msg[30];
extern char 				value[60];


extern CAN_HandleTypeDef 	hcan;

extern TIM_HandleTypeDef 	htim4;
extern TIM_HandleTypeDef 	htim3;
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

/*CUSTOM FUNCTIONS*/

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
		Error_Handler();
    }

    /*Reboot Board - Received command byte from CAN*/
	else if ((RxHeader.StdId == BOOTLOADER_ID_CAN) && (RxHeader.DLC == 2) && (RxData[0] == 0xFF) && (RxData[1] == 0x00))
	{
     	NVIC_SystemReset();
     	
	}

	/*Received TLB error byte in order to turn LEDs on or off*/
	else if ((RxHeader.StdId == TLB_ERROR_ID_CAN) && (RxHeader.DLC == 1) && (RxData[0] < 16))
	{
     	NOHV = (bool)(RxData[0] & 1);
     	BMS_ERR = (bool)(RxData[0] & 2);
     	IMD_ERR = (bool)(RxData[0] & 4);
     	ERR_BYTE_RECEIVED = true;
     	
	}
	/*Ready to drive ACK from DSPACE*/
	else if ((RxHeader.StdId == ACK_RTD_ID_CAN) && (RxHeader.DLC == 1))
	{
		if(RxData[0] == 1)
		{
			CTOR_EN_ACK = true;
		}
		else if(RxData[0] == 2)
		{
			RTD_EN_ACK = true;
		}	
     	
	}
	/*Set duty cycle*/
	else if ((RxHeader.StdId == PWM_ID_CAN) && (RxHeader.DLC == 2) && (RxData[0] <= 100) && (RxData[1] <= 100))
	{

		PWM_PUMP = RxData[0];
		PWM_RAD_FAN = RxData[1];
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_PUMP);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_RAD_FAN);


	}

}

/*FSM*/
void ReadyToDriveFSM(uint32_t delay_100us)
{
	static uint32_t delay_100us_last = 0;

	if(delay_fun(&delay_100us_last,delay_100us))
	{
		static int counter_buzzer = 0;

		switch (rtd_fsm)
		{
			case IDLE :
				if(HAL_GPIO_ReadPin(RTD_BUTTON_GPIO_Port, RTD_BUTTON_Pin))
				{
					rtd_fsm = CTOR_EN;
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
				if(CTOR_EN_ACK)
				{
					CTOR_EN_ACK = false;
					rtd_fsm = RTD_EN;
				}
			break;

			case RTD_EN :
				if(HAL_GPIO_ReadPin(RTD_BUTTON_GPIO_Port, RTD_BUTTON_Pin))
				{
					TxHeader.StdId = CMD_RTD_ID_CAN;
					TxHeader.ExtId = CMD_RTD_ID_CAN;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.IDE = CAN_ID_STD;
					TxHeader.DLC = 1;
					TxHeader.TransmitGlobalTime = DISABLE;
					TxData[0]= 0x2;
					CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
					rtd_fsm = WAIT_RTD_EN_ACK;
				}
			break;

			case WAIT_RTD_EN_ACK :
				if(RTD_EN_ACK)
				{
					RTD_EN_ACK = false;
					rtd_fsm = RTD;
				}
			break;

			case RTD :
			if(counter_buzzer < 10)
			{
				counter_buzzer++;
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, ON);
			}
			else
			{
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, OFF);
				rtd_fsm = STOP;
			}

			break;

			case STOP:
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
			HAL_GPIO_WritePin(GPIOE, BMS_LED_Pin, BMS_ERR);
			HAL_GPIO_WritePin(GPIOE, NOHV_LED_Pin, NOHV);
			HAL_GPIO_WritePin(GPIOE, IMD_LED_Pin, IMD_ERR);

			/*LED ON or OFF depending on ERR_BYTE*/

		}
		else
		{
			HAL_GPIO_WritePin(GPIOE, BMS_LED_Pin, ON);
			HAL_GPIO_WritePin(GPIOE, NOHV_LED_Pin, ON);
			HAL_GPIO_WritePin(GPIOE, IMD_LED_Pin, ON);
			/*LED always ON, timeout can*/
		}

	}

}



/*Setup TIMER, CAN*/
void SetupDashBoard(void)
{


	HAL_TIM_Base_Start_IT(&htim3);
	/*Start timer in IT mode for counter*/

	/*Start timer CH2 for PWM*/
	if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
  	{
    /* PWM generation Error */
    	Error_Handler();
  	}

  	/*Start timer CH3 for PWM*/
	if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
  	{
    /* PWM generation Error */
    	Error_Handler();
  	}

	CAN_Config();
  	/*Start Can peripheral*/

	sprintf(msg, "DashBoard Boot - v0.1\n\r");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, 30, 20);
	memset(msg,0,strlen(msg));
	sprintf(msg, "Configuration complete\n\r\n\r");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, 30, 20);
	

	HAL_GPIO_WritePin(GPIOE, BMS_LED_Pin, ON);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOE, BMS_LED_Pin, OFF);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOE, NOHV_LED_Pin, ON);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOE, NOHV_LED_Pin, OFF);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOE, IMD_LED_Pin, ON);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOE, IMD_LED_Pin, OFF);
	HAL_Delay(500);
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

			Error_Handler();

		}

	}

	if (HAL_CAN_AddTxMessage(hcan, pHeader, aData, pTxMailbox) != HAL_OK)
    {
    
        /* Transmission request Error */
          Error_Handler();
    
    }


}

/*Send data to CAN BUS*/
void CAN_Tx(uint32_t delay_100us)
{
	static uint32_t delay_100us_last = 0;

	if(delay_fun(&delay_100us_last,delay_100us))
	{
		TxHeader.StdId = 0x21;
		TxHeader.ExtId = 0x021;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.DLC = 8;
		TxHeader.TransmitGlobalTime = DISABLE;
		/*
		TxData[0]= (ADC_OUT[0] >> 8);
		TxData[1]= ADC_OUT[0];

		TxData[2]= (ADC_OUT[1] >> 8);
		TxData[3]= ADC_OUT[1];

		TxData[4]= (ADC_OUT[2] >> 8);
		TxData[5]= ADC_OUT[2];

		TxData[6]= (ADC_OUT[3] >> 8);
		TxData[7]= ADC_OUT[3];
		*/
		CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
        /*Message 1 with 4 ADC_IN*/

		
		
	}

}


/*Print on uart */
void Debug_UART(int state)
{
	

}


/*Core SensorBoard*/
void CoreDashBoard(void)
{

	UpdateCockpitLed(5000);
	/*Update state Cockpit's LEDs*/

	ReadyToDriveFSM(1000);
	/*Ready to drive FSM*/

	//HAL_Delay(2000);
	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, i++);

	
	//Debug_UART(debug);
	/*Print into "value" string the content of ADC[], then write to UART1*/


	//CAN_Tx(20);
	/*Send data via CAN*/
	

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if((*htim).Instance == TIM3)
	{
		counter++;
	}
}








