/*INCLUDES*/

#include "main.h"
#include "stm32f3xx_hal.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

/*CUSTOM DEFINE*/

#define ON 1
#define OFF 0
#define BRAKE_THRESHOLD 500
#define DASH_STATUS 0x45
#define DEBUG

typedef enum
{
    ERROR_NONE = 0,
    ERROR_INIT_BTN,
    ERROR_ILLEGAL_RTD_BTN,
    ERROR_BRAKE_PRESSURE
} error_t;

/*CUSTOM FUNCTIONS PROTOTYPES*/

void SetupDashBoard(void);
void CoreDashBoard(void);
void can_send_state(uint32_t delay_100us);
void UpdateCockpitLed(uint32_t delay_100us);
void ReadyToDriveFSM(uint32_t delay_100us);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

extern void Error_Handler(void);