/*INCLUDE*/

#include "dashboard.h"
#include "button.h"
#include "can.h"
#include "sc22_evo_canlv.h"
#include "tim.h"
#include "usart.h"
#include "utils.h"
#include "wdg.h"
#include <stdio.h>

/* Dashboard State Machine */
typedef enum
{
    STATE_IDLE = 0,
    STATE_CTOR_EN_WAIT_ACK,
    STATE_TS_ON,
    STATE_RTD_WAIT_ACK,
    STATE_RTD,
    STATE_IDLE_WAIT_ACK,
    STATE_ERROR
} state;

/* State change triggers */
typedef enum
{
    TRIG_NONE = 0, // No trigger/triggered by CAN bus
    TRIG_COCK = 1, // Cockpit button
    TRIG_EXT = 2   // External button
} state_trig;
state_trig STATE_CHANGE_TRIG = TRIG_NONE;

// FSM state keeping
state rtd_fsm = STATE_IDLE;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;
uint8_t TxData[8] = {0};
uint8_t RxData[8] = {0};

/* Error Variables */
error_t error = ERROR_NONE;
uint8_t boards_timeouts;

/* Cock LEDs flags */
volatile bool SD_CLOSED;
volatile bool BMS_ERR;
volatile bool TSOFF;
volatile bool IMD_ERR;

/* dSpace ACK flags */
volatile bool CTOR_EN_ACK;
volatile bool RTD_EN_ACK;
volatile bool IDLE_ACK;
volatile bool NACK;

/* PWM Variables */
volatile uint8_t PWM_BAT_FAN;
volatile uint8_t PWM_ASB_MOTOR;
#if PCBVER == 2
volatile uint8_t PWM_RADIATOR_FAN;
#elif PCBVER == 1
volatile uint8_t PWM_POWERTRAIN;
#endif
volatile float BRAKE_EXT;

/* Button short press flags */
bool RTD_BUTTON = false;

/*CUSTOM FUNCTIONS*/

/*Rx Message interrupt from CAN*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    union {
        struct sc22_evo_canlv_d_space_rtd_ack_t rtd_ack;
        struct sc22_evo_canlv_d_space_peripherals_ctrl_t per_ctrl;
        struct sc22_evo_canlv_sens_front_1_t sens_front_1;
        struct sc22_evo_canlv_tlb_battery_tsal_status_t tsal_status;
        struct sc22_evo_canlv_tlb_battery_shut_status_t shut_status;
    } msgs;

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
    case SC22_EVO_CANLV_D_SPACE_RTD_ACK_FRAME_ID:
    case SC22_EVO_CANLV_D_SPACE_PERIPHERALS_CTRL_FRAME_ID:
        // Reset dSpace timeout after boot
        wdg_timeouts_100us[WDG_BOARD_DSPACE] = 4800;
        wdg_reset(WDG_BOARD_DSPACE, now);
        break;
    case SC22_EVO_CANLV_TLB_BATTERY_TSAL_STATUS_FRAME_ID:
    case SC22_EVO_CANLV_TLB_BATTERY_SHUT_STATUS_FRAME_ID:
        wdg_reset(WDG_BOARD_TLB, now);
        break;
    case SC22_EVO_CANLV_SENS_FRONT_1_FRAME_ID:
        wdg_reset(WDG_BOARD_SENS_FRONT, now);
        break;
    }

    /*Reboot Board - Received command byte from CAN*/
    if ((RxHeader.StdId == 0x9) && (RxHeader.DLC == 2) && (RxData[0] == 0xFF) && (RxData[1] == 0x00))
    {
        NVIC_SystemReset();
    }

    /*
     *
     * sensFront
     *
     */
    else if((RxHeader.StdId == SC22_EVO_CANLV_SENS_FRONT_1_FRAME_ID) && (RxHeader.DLC == SC22_EVO_CANLV_SENS_FRONT_1_LENGTH)) {
        sc22_evo_canlv_sens_front_1_unpack(&msgs.sens_front_1, RxData, SC22_EVO_CANLV_SENS_FRONT_1_LENGTH);

        BRAKE_EXT = sc22_evo_canlv_sens_front_1_brake_straingauge_voltage_m_v_decode(msgs.sens_front_1.brake_straingauge_voltage_m_v);
    }
    /*
     *
     * dSpace
     *
     */
    else if ((RxHeader.StdId == SC22_EVO_CANLV_D_SPACE_RTD_ACK_FRAME_ID) && (RxHeader.DLC == SC22_EVO_CANLV_D_SPACE_RTD_ACK_LENGTH))
    {
        sc22_evo_canlv_d_space_rtd_ack_unpack(&msgs.rtd_ack, RxData, SC22_EVO_CANLV_D_SPACE_RTD_ACK_LENGTH);

        if(msgs.rtd_ack.ctor_en_ack) {
            CTOR_EN_ACK = true;
        } else if(msgs.rtd_ack.rtd_en_ack) {
            RTD_EN_ACK = true;
        } else if(msgs.rtd_ack.reboot_fsm) {
            IDLE_ACK = true;
            NACK = true;
        }
    }
    else if ((RxHeader.StdId == SC22_EVO_CANLV_D_SPACE_PERIPHERALS_CTRL_FRAME_ID) && (RxHeader.DLC == SC22_EVO_CANLV_D_SPACE_PERIPHERALS_CTRL_LENGTH))
    {
#if PCBVER == 2
        sc22_evo_canlv_d_space_peripherals_ctrl_unpack(&msgs.per_ctrl, RxData, SC22_EVO_CANLV_D_SPACE_PERIPHERALS_CTRL_LENGTH);
        
        PWM_RADIATOR_FAN = (msgs.per_ctrl.rad_fan_pwm_ctrl/255.*__HAL_TIM_GetAutoreload(&RADIATOR_FANS_PWM_TIM));
        __HAL_TIM_SET_COMPARE(&RADIATOR_FANS_PWM_TIM, RADIATOR_FANS_PWM_CH, PWM_RADIATOR_FAN);
#endif
        PWM_BAT_FAN = (msgs.per_ctrl.batt_hv_fan_ctrl/255.*__HAL_TIM_GetAutoreload(&RADIATOR_FANS_PWM_TIM));
        __HAL_TIM_SET_COMPARE(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH, PWM_BAT_FAN);
    }

    /*
     *
     * TLB
     *
     */
    /* Received TLB error byte in order to turn LEDs on or off */
    else if ((RxHeader.StdId == SC22_EVO_CANLV_TLB_BATTERY_TSAL_STATUS_FRAME_ID) && (RxHeader.DLC == SC22_EVO_CANLV_TLB_BATTERY_TSAL_STATUS_LENGTH))
    {
        sc22_evo_canlv_tlb_battery_tsal_status_unpack(&msgs.tsal_status, RxData, SC22_EVO_CANLV_TLB_BATTERY_TSAL_STATUS_LENGTH);
        
        TSOFF = (bool)msgs.tsal_status.tsal_is_green_on;
    }
    else if ((RxHeader.StdId == SC22_EVO_CANLV_TLB_BATTERY_SHUT_STATUS_FRAME_ID) && (RxHeader.DLC == SC22_EVO_CANLV_TLB_BATTERY_SHUT_STATUS_LENGTH)) {
        sc22_evo_canlv_tlb_battery_shut_status_unpack(&msgs.shut_status, RxData, SC22_EVO_CANLV_TLB_BATTERY_SHUT_STATUS_LENGTH);
        BMS_ERR = (bool)msgs.shut_status.is_ams_error_latched;
        IMD_ERR = (bool)msgs.shut_status.is_imd_error_latched;
        SD_CLOSED = (bool)msgs.shut_status.is_shutdown_closed_pre_tlb_batt_final;
    }
}

void InitDashBoard()
{
    // Turn on all LEDs
    HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD_CLOSED_CMD_GPIO_Port, SD_CLOSED_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AMS_ERR_CMD_GPIO_Port, AMS_ERR_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMD_ERR_CMD_GPIO_Port, IMD_ERR_CMD_Pin, GPIO_PIN_SET);

    HAL_Delay(900);
    HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_RESET);

    // Test inputs
    // if (button_get(BUTTON_RTD))
    // {
    //     error = ERROR_INIT_BTN;
    //     rtd_fsm = STATE_ERROR;
    // }
}

void cock_callback()
{
    RTD_BUTTON = true;
}

/*FSM*/
void ReadyToDriveFSM(uint32_t delay_100us)
{
    // Set first delay back to give time for the buttons to debounce
    static uint32_t delay_100us_last = 0;
    static uint32_t timeout = 0;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        // static int counter_buzzer = 0;

        switch (rtd_fsm)
        {
        case STATE_IDLE:
            if (RTD_BUTTON)
            {
                STATE_CHANGE_TRIG = TRIG_COCK;
                rtd_fsm = STATE_CTOR_EN_WAIT_ACK;
                timeout = HAL_GetTick();
            }
            HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, GPIO_PIN_RESET);
            break;

        case STATE_CTOR_EN_WAIT_ACK:
            if (NACK || HAL_GetTick() - timeout > 500)
            {
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_IDLE;
            }
            else if (CTOR_EN_ACK)
            {
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_TS_ON;
                timeout = HAL_GetTick();
            }
            break;

        case STATE_TS_ON:
            if(NACK) {
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_IDLE;
            }
            else if (RTD_BUTTON && BRAKE_EXT > 400)
            {
                STATE_CHANGE_TRIG = TRIG_COCK;
                rtd_fsm = STATE_RTD_WAIT_ACK;
                timeout = HAL_GetTick();
            }
            break;

        case STATE_RTD_WAIT_ACK:
            if (NACK || IDLE_ACK || HAL_GetTick() - timeout > 500)
            {
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_IDLE;
            }
            else if (RTD_EN_ACK)
            {
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_RTD;
                HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_SET);
            }
            break;

        case STATE_RTD:
            // TODO: buzzer
            if (IDLE_ACK)
            {
                rtd_fsm = STATE_IDLE;
            }
            else if (RTD_BUTTON)
            {
                STATE_CHANGE_TRIG = TRIG_COCK;
                rtd_fsm = STATE_IDLE_WAIT_ACK;
            }
            HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);

            if(HAL_GetTick() - timeout > 1000)
                HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_RESET);
            // if (counter_buzzer < 40)
            //{
            //     counter_buzzer++;
            //     HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, ON);
            //     HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, ON);
            //     HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
            // }
            // else
            //{
            //     HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, OFF);
            //     HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, OFF);
            //      HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
            //     counter_buzzer = 0;
            //     rtd_fsm = STATE_IDLE;
            // }
            break;

        case STATE_IDLE_WAIT_ACK:
            if (NACK)
            {
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_RTD;
            }
            else if (IDLE_ACK)
            {
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_IDLE;
            }
            break;

        case STATE_ERROR:
            // TODO: uncomment
            // Engineers around the world are thankful this line is commented
            // as_state = AS_EMERGENCY;
            break;
        }

        // Reset all flags
        CTOR_EN_ACK = false;
        RTD_EN_ACK = false;
        IDLE_ACK = false;
        NACK = false;

        RTD_BUTTON = false;
    }
}

/*Update Cockpit's LEDs*/
void UpdateCockpitLed(uint32_t delay_100us)
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        HAL_GPIO_WritePin(SD_CLOSED_CMD_GPIO_Port, SD_CLOSED_CMD_Pin, SD_CLOSED);

        if (boards_timeouts & WDG_BOARD_TLB)
        {
            // CAN timeout. everything is bad
            HAL_GPIO_WritePin(AMS_ERR_CMD_GPIO_Port, AMS_ERR_CMD_Pin, ON);
            HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, OFF);
            HAL_GPIO_WritePin(IMD_ERR_CMD_GPIO_Port, IMD_ERR_CMD_Pin, ON);
        }
        else
        {
            HAL_GPIO_WritePin(AMS_ERR_CMD_GPIO_Port, AMS_ERR_CMD_Pin, BMS_ERR);
            HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, TSOFF);
            HAL_GPIO_WritePin(IMD_ERR_CMD_GPIO_Port, IMD_ERR_CMD_Pin, IMD_ERR);
        }
    }
}

/*Setup TIMER, CAN*/
void SetupDashBoard(void)
{

/*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&RADIATOR_FANS_PWM_TIM, RADIATOR_FANS_PWM_CH) != HAL_OK)
    {
        /* PWM generation Error */
        Error_Handler();
    }

    /*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH) != HAL_OK)
    {
        /* PWM generation Error */
        Error_Handler();
    }
    button_set_shortpress_callback(BUTTON_RTD, cock_callback);

    char msg[54] = {0};
    sprintf(msg, "Dashboard 2022 Boot - build %s @ %s\n\r", __DATE__, __TIME__);
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, 38, 20);
}

/*Send status data to CAN BUS*/
void can_send_state(uint32_t delay_100us)
{
    static uint32_t delay_100us_last = 0;

    union {
        struct sc22_evo_canlv_steering_rtd_t rtd;
        struct sc22_evo_canlv_steering_motor_control_debug_t motor_ctrl_dbg;
    } msgs;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        switch(rtd_fsm) {
            case STATE_IDLE:
                msgs.rtd.rtd_cmd = 0x0;
                break;
            case STATE_CTOR_EN_WAIT_ACK:
                msgs.rtd.rtd_cmd = 0x1;
                break;
            case STATE_RTD_WAIT_ACK:
                msgs.rtd.rtd_cmd = 0x2;
                break;
            default:
                msgs.rtd.rtd_cmd = rtd_fsm;
                break;
        }
        sc22_evo_canlv_steering_rtd_pack(TxData, &msgs.rtd, SC22_EVO_CANLV_STEERING_RTD_LENGTH);

        TxHeader.StdId = SC22_EVO_CANLV_STEERING_RTD_FRAME_ID;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = SC22_EVO_CANLV_STEERING_RTD_LENGTH;

        CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 300);
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
    UpdateCockpitLed(1000);

    // Update buttons state
    button_sample();

    // RUN the ready to drive FSM
    ReadyToDriveFSM(500);

    // Run the AS FSM
    // mission_run();
    // as_run();

    uint8_t timeouts = wdg_check();
    if (rtd_fsm != STATE_ERROR && timeouts != 0)
    {
        error = ERROR_CAN_WDG;
        boards_timeouts = timeouts;
        rtd_fsm = STATE_ERROR;
    }

    // Send current state via CAN
    can_send_state(500);
}
