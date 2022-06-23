#include "mission.h"
#include "button.h"
#include "utils.h"

#define PERIOD_2HZ_100us 2500 // Duty cycle time to get 2Hz (toggle at 4Hz)

// Maps logic values for missions to hardware indexes for the multiplexer
static const uint8_t ami_mapping[NUM_MISSIONS] = {
    [MISSION_ACCEL] = 1,
    [MISSION_SKIDPAD] = 0,
    [MISSION_AUTOX] = 5,
    [MISSION_TRACKDRIVE] = 4,
    [MISSION_EBSTEST] = 6,
    [MISSION_INSPECT] = 3,
    [MISSION_MANUAL] = 2,
    [MISSION_NO] = 7};

static bool confirmed = false;
static mission_t current_mission = MISSION_NO;

void ami_confirm()
{
    if (mission_get() != MISSION_NO)
        confirmed = true;
}

bool mission_is_confirmed()
{
    return confirmed;
}

void ami_mission_increment()
{
    if (!confirmed)
    {
        mission_set((mission_get() % (NUM_MISSIONS - 1)) + 1);
    }
}

void mission_setup()
{
    button_set_longpress(BUTTON_MISSION, ami_confirm);
    button_set_shortpress(BUTTON_MISSION, ami_mission_increment);
}

mission_t mission_get()
{
    return current_mission;
}

void mission_set(mission_t mission)
{
    if (!confirmed)
        current_mission = mission;
}

void mission_run()
{
    static uint32_t delay_100us_last = 0;

    if (!confirmed && delay_fun(&delay_100us_last, PERIOD_2HZ_100us))
    {
        // Blink AMI LEDs
        HAL_GPIO_TogglePin(AMI_OFF_CMD_GPIO_Port, AMI_OFF_CMD_Pin);
    }
    else if (confirmed)
    {
        // Turn on AMI board
        HAL_GPIO_WritePin(AMI_OFF_CMD_GPIO_Port, AMI_OFF_CMD_Pin, GPIO_PIN_RESET);
    }

    uint8_t led_index = ami_mapping[current_mission];
    // AMI1,2 and 3 are select pins for a MUX that controls the LEDs
    HAL_GPIO_WritePin(AMI1_CMD_GPIO_Port, AMI1_CMD_Pin, (0b001 & led_index));
    HAL_GPIO_WritePin(AMI3_CMD_GPIO_Port, AMI3_CMD_Pin, (0b010 & led_index) >> 1);
    HAL_GPIO_WritePin(AMI2_CMD_GPIO_Port, AMI2_CMD_Pin, (0b100 & led_index) >> 2);
}