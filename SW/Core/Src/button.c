
#include "button.h"
#include "main.h"
#include "utils.h"

// Map GPIOs to button index
const GPIO_TypeDef *button_gpio[BUTTON_COUNT] = {TS_CK_STM_GPIO_Port, TS_EX_STM_GPIO_Port, SpareButton_STM_GPIO_Port};
const uint16_t button_pin[BUTTON_COUNT] = {TS_CK_STM_Pin, TS_EX_STM_Pin, SpareButton_STM_Pin};

// State and last change time of each button
bool state[BUTTON_COUNT] = {BUTTON_PRESSED, BUTTON_PRESSED, BUTTON_PRESSED};
uint32_t change_time[BUTTON_COUNT] = {0};

void button_sample()
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, BUTTON_SAMPLE_TIME_100us))
    {
        for (uint8_t i = 0; i < BUTTON_COUNT; i++)
        {
            if (state[i] != HAL_GPIO_ReadPin((GPIO_TypeDef *)button_gpio[i], button_pin[i]))
            {
                // If state changed, then update change_time
                change_time[i] = ReturnTime_100us();
            }
            else
            {
                if (ReturnTime_100us() - change_time[i] >= BUTTON_DEBOUNCE_TIME_100us)
                {
                    // If state hasn't changed for the last DEBOUNCE_TIME, then we toggle the state
                    state[i] = !state[i];
                }
            }
        }
    }
}

/**
 * @brief Returns the logical state of the button
 *
 * @param btn button index
 * @return true if button is PRESSED
 * @return false if button is RELEASED
 */
bool button_get(button btn)
{
    return state[btn] == BUTTON_PRESSED;
}

uint32_t button_last_change_time(button btn)
{
    return change_time[btn];
}