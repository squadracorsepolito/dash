
#include "as_fsm.h"
#include "main.h"
#include "utils.h"

#define PERIOD_4HZ_us 250000
as_state_t as_state = AS_OFF;

void as_run()
{
    switch (as_state)
    {
    case AS_OFF:
        HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_RESET);
        break;
    case AS_READY:
        HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_RESET);
        break;
    case AS_DRIVING:
        HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_SET);
        LedBlinking(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, PERIOD_4HZ_us);
        HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_RESET);
        break;
    case AS_EMERGENCY:
        LedBlinking(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, PERIOD_4HZ_us);
        HAL_GPIO_WritePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_SET);
        break;
    case AS_FINISHED:
        HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_RESET);
        break;
    }
}