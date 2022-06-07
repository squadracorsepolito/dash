/**
 * @file ami.c
 * @author Matteo Bonora (matteo.bonora@studenti.polito.it)
 * @brief Handle the AMI (autonomous mission indicator) lights.
 * @date 2022-05-08
 */
#include "ami.h"
#include "button.h"
#include "utils.h"

#define PERIOD_2HZ_100us 2500 // Duty cycle time to get 2Hz (toggle at 4Hz)

bool ami_selected = false;
static mission_t current_mission = MISSION_NO;

void ami_select()
{
    ami_selected = true;
}

void ami_increment()
{
    if (!ami_selected)
    {
        ami_set_mission((ami_get() + 1) % NUM_MISSIONS);
    }
}

void ami_setup()
{
    button_set_longpress(BUTTON_Spare, ami_select);
    button_set_shortpress(BUTTON_Spare, ami_increment);
}

mission_t ami_get()
{
    return current_mission;
}

void ami_set_mission(mission_t mission)
{
    current_mission = mission;

    // AMI1,2 and 3 are select pins for a MUX that controls the LEDs
    HAL_GPIO_WritePin(AMI1_CMD_GPIO_Port, AMI1_CMD_Pin, (0b001 & current_mission));
    HAL_GPIO_WritePin(AMI3_CMD_GPIO_Port, AMI3_CMD_Pin, (0b010 & current_mission) >> 1);
    HAL_GPIO_WritePin(AMI2_CMD_GPIO_Port, AMI2_CMD_Pin, (0b100 & current_mission) >> 2);
}

void ami_run()
{
    static uint32_t delay_100us_last = 0;

    if (!ami_selected && delay_fun(&delay_100us_last, PERIOD_2HZ_100us))
    {
        // HAL_GPIO_TogglePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin);
        HAL_GPIO_TogglePin(AMI_OFF_CMD_GPIO_Port, AMI_OFF_CMD_Pin);
    }
    else if (ami_selected)
    {
        // HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AMI_OFF_CMD_GPIO_Port, AMI_OFF_CMD_Pin, GPIO_PIN_RESET);
    }
}