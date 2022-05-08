
#include "ami.h"

mission_t current_mission = MISSION_NO;

mission_t ami_get()
{
    return current_mission;
}

void ami_set(mission_t mission)
{
    current_mission = mission;

    // AMI1,2 and 3 are select pins for a MUX that controls the LEDs
    HAL_GPIO_WritePin(AMI1_CMD_GPIO_Port, AMI1_CMD_Pin, (0b100 & current_mission) >> 2);
    HAL_GPIO_WritePin(AMI2_CMD_GPIO_Port, AMI2_CMD_Pin, (0b010 & current_mission) >> 1);
    HAL_GPIO_WritePin(AMI3_CMD_GPIO_Port, AMI3_CMD_Pin, (0b001 & current_mission));
}