
#include "utils.h"

/*Toggle a specific GPIO*/
void LedBlinking(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t period)
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, period))
    {
        HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
    }
}

/*Return true if the delay is completed, false otherwise*/
int delay_fun(uint32_t *delay_100us_last, uint32_t delay_100us)
{

    uint32_t current_time = ReturnTime_100us();

    if (current_time > *delay_100us_last && (current_time - *delay_100us_last) >= delay_100us)
    {
        *delay_100us_last = current_time;
        return 1;
    }
    else if (current_time < *delay_100us_last && (0xFFFFFFFF - current_time - *delay_100us_last) >= delay_100us)
    {
        *delay_100us_last = current_time;
        return 1;
    }
    /*In case of timer overflow, the delay is computed correctly*/

    return 0;
}

/*Return the value of the counter that is incremented every 100us*/
uint32_t ReturnTime_100us(void)
{
    return counter;
}