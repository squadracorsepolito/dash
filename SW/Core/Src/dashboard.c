/*INCLUDE*/

#include "dashboard.h"
#include "as_fsm.h"
#include "button.h"
#include "can.h"
#include "can_watchdog.h"
#include "mission.h"
#include "tim.h"
#include "usart.h"
#include "utils.h"
#include <stdio.h>
/*EXTERNAL GLOBAL VARIABLES*/

extern char msg[80];
extern char value[60];

extern CAN_HandleTypeDef hcan;

extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint32_t TxMailbox;
extern uint8_t TxData[8];
extern uint8_t RxData[8];

/*Error Variables*/
error_t error = ERROR_NONE;
uint8_t boards_timeouts;

extern bool ASB_ERR; // Autonomous System Brake
extern bool BMS_ERR;
extern bool NOHV; // TS Off
extern bool IMD_ERR;
extern bool TLB_ERR_RECEIVED;

/*PWM Variables*/
extern uint8_t PWM_RAD_FAN;
extern uint8_t PWM_PUMP;
extern uint8_t PWM_BP_FAN;

/*State Machine for RTD*/
typedef enum
{
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_CTOR_EN,
    STATE_WAIT_CTOR_EN_ACK,
    STATE_RTD_EN,
    STATE_WAIT_RTD_EN_ACK,
    STATE_RTD,
    STATE_STOP,
    STATE_ERROR
} state;

state rtd_fsm = STATE_INIT;

/*RTD_FSM variables*/
extern bool CTOR_EN_ACK;
extern bool RTD_EN_ACK;
extern bool REBOOT_FSM;

/*Front brake pressure value*/
extern volatile uint16_t brake_pressure;

/*CUSTOM FUNCTIONS*/

/*Rx Message interrupt from CAN*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        /* Transmission request Error */
        HAL_CAN_ResetError(hcan);
        Error_Handler();
    }

    // Reset watchdog
    uint32_t now = ReturnTime_100us();
    switch (RxHeader.StdId)
    {
    case AS_STATE_ID_CAN:
    case ACK_RTD_ID_CAN:
    case CMD_EBS_ID_CAN:
        wdg_reset(WDG_DSPACE, now);
        break;
    case TLB_ERROR_ID_CAN:
        wdg_reset(WDG_TLB, now);
        break;
    case SENSORBOARD_4_7_ID_CAN:
        wdg_reset(WDG_SENSORS, now);
        break;
    }

    /*Reboot Board - Received command byte from CAN*/
    if ((RxHeader.StdId == BOOTLOADER_ID_CAN) && (RxHeader.DLC == 2) && (RxData[0] == 0xFF) && (RxData[1] == 0x00))
    {
        NVIC_SystemReset();
    }
    /*Received TLB error byte in order to turn LEDs on or off*/
    else if ((RxHeader.StdId == TLB_ERROR_ID_CAN) && (RxHeader.DLC == 1) && (RxData[0] < 16))
    {
        NOHV = (bool)(RxData[0] & 1);
        BMS_ERR = (bool)(RxData[0] & 4) || (bool)(RxData[0] & 2);
        IMD_ERR = (bool)(RxData[0] & 4);
        TLB_ERR_RECEIVED = true;
    }
    /* AS state */
    else if ((RxHeader.StdId == AS_STATE_ID_CAN) && (RxHeader.DLC == 1))
    {
        ASB_ERR = (bool)(RxData[0] & (1 << 7));
        as_state = RxData[0];
    }
    /*Ready to drive ACK from DSPACE*/
    else if ((RxHeader.StdId == ACK_RTD_ID_CAN) && (RxHeader.DLC == 1))
    {
        if ((RxData[0] == 1) && (rtd_fsm == STATE_WAIT_CTOR_EN_ACK))
        {
            CTOR_EN_ACK = true;
        }
        else if ((RxData[0] == 2) && (rtd_fsm == STATE_WAIT_RTD_EN_ACK))
        {
            RTD_EN_ACK = true;
        }
        else if ((RxData[0] == 3))
        {
            REBOOT_FSM = true;
        }
    }
    /*EBS commmand*/
    else if ((RxHeader.StdId == CMD_EBS_ID_CAN) && (RxHeader.DLC == 1))
    {
        HAL_GPIO_WritePin(EBS_RELAY1_GPIO_Port, EBS_RELAY1_Pin, RxData[0] & 0b1);
        HAL_GPIO_WritePin(EBS_RELAY2_GPIO_Port, EBS_RELAY2_Pin, RxData[0] >> 1);
    }
    /*Set duty cycle*/
    else if ((RxHeader.StdId == PWM_ID_CAN) && (RxHeader.DLC == 3))
    {
        if (RxData[0] <= 100)
        {
            PWM_PUMP = RxData[0];
            __HAL_TIM_SET_COMPARE(&PWM_PUMP_TIM, PWM_PUMP_CH, PWM_PUMP);
        }
        if (RxData[1] <= 100)
        {
            PWM_RAD_FAN = RxData[1];
            __HAL_TIM_SET_COMPARE(&PWM_RAD_TIM, PWM_RAD_CH, PWM_RAD_FAN);
        }
        if (RxData[2] <= 100)
        {
            PWM_BP_FAN = RxData[2];
            __HAL_TIM_SET_COMPARE(&PWM_BP_TIM, PWM_BP_CH, PWM_BP_FAN);
        }
    }
    else if ((RxHeader.StdId == SENSORBOARD_4_7_ID_CAN) && (RxHeader.DLC == 8))
    {
        brake_pressure = ((RxData[6] << 8) | RxData[7]);
    }
}

/*FSM*/
void ReadyToDriveFSM(uint32_t delay_100us)
{
    // Set first delay back to give time for the buttons to debounce
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        static int counter_buzzer = 0;

        switch (rtd_fsm)
        {
        case STATE_INIT:
        {
            static uint8_t substate = 0;
            static uint32_t last = 0;

            // Micro state machine to handle sleeps
            switch (substate)
            {
            case 0:
                HAL_GPIO_WritePin(EBS_RELAY1_GPIO_Port, EBS_RELAY1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(EBS_RELAY2_GPIO_Port, EBS_RELAY2_Pin, GPIO_PIN_RESET);

                // Turn on all LEDs
                HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(ASB_CMD_GPIO_Port, ASB_CMD_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(AMS_CMD_GPIO_Port, AMS_CMD_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(IMD_CMD_GPIO_Port, IMD_CMD_Pin, GPIO_PIN_SET);

                as_state = AS_TEST;

                if (ReturnTime_100us() - last >= 8000)
                {
                    last = ReturnTime_100us();
                    substate++;
                };
                break;

            case 1:
                // Beep!
                HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_SET);

                if (ReturnTime_100us() - last >= 1000)
                {
                    last = ReturnTime_100us();
                    substate++;
                };
                break;
            case 2:
                HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_SET);

                if (ReturnTime_100us() - last >= 1000)
                {
                    last = ReturnTime_100us();
                    substate++;
                };
                break;
            case 3:
                HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_RESET);

                // Test input states
                if (button_get(BUTTON_TS_CK) ||
                    button_get(BUTTON_TS_EX) ||
                    button_get(BUTTON_MISSION))
                {
                    error = ERROR_INIT_BTN;
                    rtd_fsm = STATE_ERROR;
                }

                as_state = AS_OFF;
                substate = 0;
                rtd_fsm = STATE_IDLE;
                break;
            }
        }
        break;
        case STATE_IDLE:
            if (button_get(BUTTON_TS_EX) || button_get(BUTTON_TS_CK))
            {
                rtd_fsm = STATE_CTOR_EN;
            }
            else
            {
                TxHeader.StdId = DASH_RTD_ID_CAN;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.DLC = 1;
                TxHeader.TransmitGlobalTime = DISABLE;
                TxData[0] = 0x0;
                CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
            }
            break;

        case STATE_CTOR_EN:

            TxHeader.StdId = DASH_RTD_ID_CAN;
            TxHeader.RTR = CAN_RTR_DATA;
            TxHeader.IDE = CAN_ID_STD;
            TxHeader.DLC = 1;
            TxHeader.TransmitGlobalTime = DISABLE;
            TxData[0] = 0x1;
            CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);

            rtd_fsm = STATE_WAIT_CTOR_EN_ACK;

            break;

        case STATE_WAIT_CTOR_EN_ACK:

            if (CTOR_EN_ACK && REBOOT_FSM == false)
            {
                CTOR_EN_ACK = false;
                rtd_fsm = STATE_RTD_EN;
            }
            else if (REBOOT_FSM)
            {
                REBOOT_FSM = false;
                HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, OFF);
                rtd_fsm = STATE_IDLE;
            }
            else
            {
                TxHeader.StdId = DASH_RTD_ID_CAN;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.DLC = 1;
                TxHeader.TransmitGlobalTime = DISABLE;
                TxData[0] = 0x1;
                CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
            }
            break;

        case STATE_RTD_EN:
            if (button_get(BUTTON_TS_EX) || button_get(BUTTON_TS_CK))
            {
                if (brake_pressure >= BRAKE_THRESHOLD)
                {
                    // CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
                    rtd_fsm = STATE_WAIT_RTD_EN_ACK;
                }
                else
                {
                    // TODO: send error instead of locking up?
                    error = ERROR_BRAKE_PRESSURE;
                    rtd_fsm = STATE_ERROR;
                }
            }
            else if (REBOOT_FSM)
            {
                REBOOT_FSM = false;
                HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, OFF);
                rtd_fsm = STATE_IDLE;
            }
            break;

        case STATE_WAIT_RTD_EN_ACK:

            if (RTD_EN_ACK)
            {
                RTD_EN_ACK = false;
                rtd_fsm = STATE_RTD;
            }
            else if (REBOOT_FSM)
            {
                REBOOT_FSM = false;
                HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, OFF);
                rtd_fsm = STATE_IDLE;
            }
            else
            {
                TxHeader.StdId = DASH_RTD_ID_CAN;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.DLC = 1;
                TxHeader.TransmitGlobalTime = DISABLE;
                TxData[0] = 0x2;
                CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
            }
            break;

        case STATE_RTD:
            if (counter_buzzer < 40)
            {
                counter_buzzer++;
                HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, ON);
                HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, ON);
                HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
            }
            else
            {
                HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, OFF);
                HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, OFF);
                // HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
                counter_buzzer = 0;
                rtd_fsm = STATE_STOP;
            }

            break;

        case STATE_STOP:
            if (REBOOT_FSM)
            {
                HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, OFF);
                TxHeader.StdId = DASH_RTD_ID_CAN;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.DLC = 1;
                TxHeader.TransmitGlobalTime = DISABLE;
                TxData[0] = 0x0;
                CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);

                REBOOT_FSM = false;
                rtd_fsm = STATE_IDLE;
            }
            break;

        case STATE_ERROR:
            TxHeader.StdId = DASH_RTD_ID_CAN;
            TxHeader.RTR = CAN_RTR_DATA;
            TxHeader.IDE = CAN_ID_STD;
            TxHeader.DLC = 2;
            TxHeader.TransmitGlobalTime = DISABLE;
            TxData[0] = error;
            TxData[1] = 0;
            switch (error)
            {
            case ERROR_CAN_WDG:
                TxData[1] = boards_timeouts;
                break;
            default:
                break;
            }
            CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
            break;
        }
    }
}

/*Update Cockpit's LEDs*/
void UpdateCockpitLed(uint32_t delay_100us)
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        HAL_GPIO_WritePin(ASB_CMD_GPIO_Port, ASB_CMD_Pin, ASB_ERR);

        if (TLB_ERR_RECEIVED)
        {
            TLB_ERR_RECEIVED = false;
            HAL_GPIO_WritePin(AMS_CMD_GPIO_Port, AMS_CMD_Pin, BMS_ERR);
            HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, NOHV);
            HAL_GPIO_WritePin(IMD_CMD_GPIO_Port, IMD_CMD_Pin, IMD_ERR);

            /*LED ON or OFF depending on ERR_RECEIVED*/
        }
        else
        {
            HAL_GPIO_WritePin(AMS_CMD_GPIO_Port, AMS_CMD_Pin, ON);
            HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, OFF);
            HAL_GPIO_WritePin(IMD_CMD_GPIO_Port, IMD_CMD_Pin, ON);
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

    mission_setup();

    sprintf(msg, "Dashboard 2022 Boot - build %s @ %s\n\r", __DATE__, __TIME__);
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, 30, 20);
    memset(msg, 0, strlen(msg));
    sprintf(msg, "Configuration complete\n\r\n\r");
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, 30, 20);

#ifdef DEBUG
    memset(msg, 0, strlen(msg));
    sprintf(msg, "Debug Mode = ON\n\r\n\r");
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, 30, 20);
#endif

    /*BootScreen with verion of the code and message of complete configuration*/
}

/*Send status data to CAN BUS*/
void can_send_state(uint32_t delay_100us)
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        TxHeader.StdId = DASH_STATUS_ID_CAN;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 5;
        TxHeader.TransmitGlobalTime = DISABLE;
        TxData[0] = 0x46;
        TxData[1] = rtd_fsm;
        TxData[2] = mission_is_confirmed() ? mission_get() : MISSION_NO;
        TxData[3] = (HAL_GPIO_ReadPin(AMS_CMD_GPIO_Port, AMS_CMD_Pin)) |
                    (HAL_GPIO_ReadPin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin) << 1) |
                    (HAL_GPIO_ReadPin(IMD_CMD_GPIO_Port, IMD_CMD_Pin) << 2) |
                    (HAL_GPIO_ReadPin(IMD_CMD_GPIO_Port, IMD_CMD_Pin) << 3) |
                    (HAL_GPIO_ReadPin(ASB_CMD_GPIO_Port, ASB_CMD_Pin) << 4);
        TxData[4] = button_get(BUTTON_TS_CK) | button_get(BUTTON_TS_EX) | button_get(BUTTON_MISSION);

        CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
    }
}

/**
 * @brief Dash main loop
 */
void CoreDashBoard(void)
{
    // Blink green led to signal activity
    static uint32_t led_blink = 0;
    LedBlinking(LED2_GPIO_Port, LED2_Pin, &led_blink, 2000);

    // Update state Cockpit's LEDs
    UpdateCockpitLed(5000);

    // Update buttons state
    button_sample();

    // RUN the ready to drive FSM
    ReadyToDriveFSM(500);

    // Run the AS FSM
    mission_run();
    as_run();

    // Send current state via CAN
    can_send_state(500);

    // boards_timeouts = wdg_check();
    // if (boards_timeouts != 0)
    //{
    //     error = ERROR_CAN_WDG;
    //     rtd_fsm = STATE_ERROR;
    // }
}
