
#include "button.h"
#include "main.h"

// Map GPIOs to button index
const GPIO_TypeDef *button_gpio[BUTTON_COUNT] = {TS_CK_STM_GPIO_Port, TS_EX_STM_GPIO_Port, SpareButton_STM_GPIO_Port};
const uint16_t button_pin[BUTTON_COUNT] = {TS_CK_STM_Pin, TS_EX_STM_Pin, SpareButton_STM_Pin};

// State and last change time of each button
bool state[BUTTON_COUNT] = {BUTTON_RELEASED};
uint32_t change_time[BUTTON_COUNT] = {0};

void button_tick()
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, BUTTON_SAMPLE_TIME_100us))
    {
        for (uint8_t i = 0; i < BUTTON_COUNT; i++)
        {
            // If state is different from input, then update state
            if (state[i] != HAL_GPIO_ReadPin((GPIO_TypeDef *)button_gpio[i], button_pin[i]))
            {
                state[i] = !state[i];
                change_time[i] = delay_100us_last;
            }
        }
    }
}

button_state button_get(button btn)
{
    // If the button has remained in its current state for at least the sample
    // time, then its current state is correct
    if (ReturnTime_100us() - change_time[btn] > BUTTON_SAMPLE_TIME_100us)
    {
        return state[btn];
    }
    // Otherwise, if the button's state has changed less than SAMPLE_TIME ago,
    // we return its previous value.
    return !state[btn];
}

uint32_t button_last_change_time(button btn)
{
    return change_time[btn];
}