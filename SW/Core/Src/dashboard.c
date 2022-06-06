/*INCLUDE*/

#include "dashboard.h"
#include "ami.h"
#include "as_fsm.h"
#include "button.h"
#include "can.h"
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

// TODO: what's this?
extern bool ASB_ERR;
extern bool BMS_ERR;
extern bool NOHV;
extern bool IMD_ERR;
extern bool TLB_ERR_RECEIVED;
bool ASMS_ON = false;

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
extern bool ASMS_ON;

/*Front brake pressure value*/
extern uint16_t brake;

/*CUSTOM FUNCTIONS*/

/*Rx Message interrupt from CAN*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        /* Transmission request Error */
        // HAL_CAN_ResetError(hcan);
        // Error_Handler();
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
        BMS_ERR = (bool)(RxData[0] & 4) || (bool)(RxData[0] & 2);
        IMD_ERR = (bool)(RxData[0] & 4);
        TLB_ERR_RECEIVED = true;
    }
    /* AS state */
    else if ((RxHeader.StdId == AS_STATE_ID_CAN) && (RxHeader.DLC == 1))
    {
        ASB_ERR = (bool)(RxData[0] & (1 << 7));

        switch (RxData[0])
        {
        case 0:
            as_state = AS_OFF;
            break;
        case 1:
            as_state = AS_READY;
            break;
        case 2:
            as_state = AS_DRIVING;
            break;
        case 3:
            as_state = AS_FINISHED;
            break;
        case 4:
            as_state = AS_EMERGENCY;
            break;
        }
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
    /*Autonomous Mission Status*/
    else if ((RxHeader.StdId == MISSION_STATUS_ID_CAN) && (RxHeader.DLC == 1))
    {
        switch (RxData[0])
        {
        case 0:
            ami_set(MISSION_NO);
            break;
        case 1:
            ami_set(MISSION_ACCEL);
            break;
        case 2:
            ami_set(MISSION_SKIDPAD);
            break;
        case 3:
            ami_set(MISSION_AUTOX);
            break;
        case 4:
            ami_set(MISSION_TRACKDRIVE);
            break;
        case 5:
            ami_set(MISSION_EBSTEST);
            break;
        case 6:
            ami_set(MISSION_INSPECT);
            break;
        case 7:
            ami_set(MISSION_MANUAL);
            break;
        }
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
        brake = ((RxData[6] << 8) | RxData[7]);
    }
}

/*FSM*/
void ReadyToDriveFSM(uint32_t delay_100us)
{
    // Set first delay back to give time for the buttons to debounce
    static uint32_t delay_100us_last = -BUTTON_DEBOUNCE_TIME_100us * 2;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        static int counter_buzzer = 0;

        switch (rtd_fsm)
        {
        case STATE_INIT:
            HAL_GPIO_WritePin(EBS_RELAY1_GPIO_Port, EBS_RELAY1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(EBS_RELAY2_GPIO_Port, EBS_RELAY2_Pin, GPIO_PIN_RESET);

            // Turn on all LEDs
            HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, GPIO_PIN_SET);
            ami_set(MISSION_NO);

            // Test input states
            if (button_get(BUTTON_TS_CK) ||
                button_get(BUTTON_TS_EX) ||
                button_get(BUTTON_Spare))
            {
                error = ERROR_INIT_BTN;
                rtd_fsm = STATE_ERROR;
            }

            // TODO: Check CAN messages

            rtd_fsm = STATE_IDLE;
            break;
        case STATE_IDLE:
            if ((button_get(BUTTON_TS_EX) && ASMS_ON) || (button_get(BUTTON_TS_CK) && !ASMS_ON))
            {
                rtd_fsm = STATE_CTOR_EN;
            }
            if ((button_get(BUTTON_TS_EX) && !ASMS_ON) || (button_get(BUTTON_TS_CK) && ASMS_ON))
            {
                error = ERROR_ILLEGAL_RTD_BTN;
                rtd_fsm = STATE_ERROR;
            }
            else
            {
                TxHeader.StdId = CMD_RTD_ID_CAN;
                TxHeader.ExtId = CMD_RTD_ID_CAN;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.DLC = 1;
                TxHeader.TransmitGlobalTime = DISABLE;
                TxData[0] = 0x0;
                CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
            }
            break;

        case STATE_CTOR_EN:

            TxHeader.StdId = CMD_RTD_ID_CAN;
            TxHeader.ExtId = CMD_RTD_ID_CAN;
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
                TxHeader.StdId = CMD_RTD_ID_CAN;
                TxHeader.ExtId = CMD_RTD_ID_CAN;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.DLC = 1;
                TxHeader.TransmitGlobalTime = DISABLE;
                TxData[0] = 0x1;
                CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
            }
            break;

        case STATE_RTD_EN:
            if ((button_get(BUTTON_TS_EX) && ASMS_ON) || (button_get(BUTTON_TS_CK) && !ASMS_ON))
            {
                if (brake >= BRAKE_THRESHOLD)
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
            if ((button_get(BUTTON_TS_EX) && !ASMS_ON) || (button_get(BUTTON_TS_CK) && ASMS_ON))
            {
                // TODO: send error instead of locking up?
                error = ERROR_ILLEGAL_RTD_BTN;
                rtd_fsm = STATE_ERROR;
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
                TxHeader.StdId = CMD_RTD_ID_CAN;
                TxHeader.ExtId = CMD_RTD_ID_CAN;
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
                HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
            }
            else
            {
                HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, OFF);
                // HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
                counter_buzzer = 0;
                rtd_fsm = STATE_STOP;
            }

            break;

        case STATE_STOP:
            if (REBOOT_FSM)
            {
                REBOOT_FSM = false;
                HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, OFF);
                rtd_fsm = STATE_IDLE;
                TxHeader.StdId = CMD_RTD_ID_CAN;
                TxHeader.ExtId = CMD_RTD_ID_CAN;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.DLC = 1;
                TxHeader.TransmitGlobalTime = DISABLE;
                TxData[0] = 0x0;
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
        TxHeader.StdId = DASH_STATUS;
        // TxHeader.ExtId = BOOTLOADER_ID_CAN;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 6;
        TxHeader.TransmitGlobalTime = DISABLE;
        TxData[0] = 0x46;
        TxData[1] = rtd_fsm;
        TxData[2] = (HAL_GPIO_ReadPin(AMS_CMD_GPIO_Port, AMS_CMD_Pin)) | (HAL_GPIO_ReadPin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin) << 1) | (HAL_GPIO_ReadPin(IMD_CMD_GPIO_Port, IMD_CMD_Pin) << 2);
        TxData[3] = PWM_PUMP;
        TxData[4] = PWM_RAD_FAN;
        TxData[5] = PWM_BP_FAN;
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
    as_run();

    // Send current state via CAN
    can_send_state(5000);
}
