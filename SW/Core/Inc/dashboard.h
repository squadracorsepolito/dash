/*INCLUDES*/

#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "stm32f3xx_hal.h"
#include "main.h"

/*CUSTOM DEFINE*/

#define ON 1
#define OFF 0
#define BOOTLOADER_ID_CAN 			0x004
#define PING_ID_CAN 				0x146
#define TLB_ERROR_ID_CAN 			0x020
#define ACK_RTD_ID_CAN 				0x040
#define PWM_ID_CAN 					0x041
#define CMD_RTD_ID_CAN 				0x042
#define CMD_COUNTER_REQUEST_ID_CAN 	0x043
#define COUNTER_REPLY_ID_CAN 		0x044
#define BRAKE_THRESHOLD				500
#define SENSORBOARD_4_7_ID_CAN		0x11
#define DASH_STATUS					0x45
#define EEPROM_ADDRESS (0x50<<1)
#define MEMORY_ADDRESS (0x01)
#define DEBUG


/*CUSTOM FUNCTIONS PROTOTYPES*/

void SetupDashBoard(void);
void CoreDashBoard(void);
uint32_t ReturnTime_100us(void);
void CAN_Config(void);
void CAN_Tx(void);
void CAN_Msg_Send(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox, uint32_t TimeOut);
void Debug_UART(int state);
int delay_fun (uint32_t *delay_100us_last, uint32_t delay_100us);
void UpdateCockpitLed(uint32_t delay_100us);
void ReadyToDriveFSM(uint32_t delay_100us);
void LedBlinking(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t period);
void Debug_CAN_Tx(uint32_t delay_100us);



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

extern void  Error_Handler(void);

