#include "button.h"
#include "main.h"
#include "utils.h"

// Map GPIOs to button index
const GPIO_TypeDef *button_gpio[BUTTON_COUNT] = {COCK_BTN_GPIO_Port, EXT_BTN_GPIO_Port, MISSION_BTN_GPIO_Port};
const uint16_t button_pin[BUTTON_COUNT] = {COCK_BTN_Pin, EXT_BTN_Pin, MISSION_BTN_Pin};

// State and last change time of each button
button_state state[BUTTON_COUNT] = {BUTTON_RELEASED, BUTTON_RELEASED, BUTTON_RELEASED};
// The last time a button changed state
uint32_t press_time[BUTTON_COUNT] = {0};
uint32_t release_time[BUTTON_COUNT] = {0};

// Callback functions to call on short and long-press events
func_type spress_func[BUTTON_COUNT] = {NULL};
func_type lpress_func[BUTTON_COUNT] = {NULL};

void button_sample()
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, BUTTON_SAMPLE_TIME_100us))
    {
        for (uint8_t i = 0; i < BUTTON_COUNT; i++)
        {
            switch (state[i])
            {
            case BUTTON_RELEASED:
                if ((button_state)HAL_GPIO_ReadPin((GPIO_TypeDef *)button_gpio[i], button_pin[i]) == BUTTON_PRESSED)
                {
                    state[i] = BUTTON_PRESSED;
                    press_time[i] = ReturnTime_100us();
                }
                break;
            case BUTTON_PRESSED:
                if ((button_state)HAL_GPIO_ReadPin((GPIO_TypeDef *)button_gpio[i], button_pin[i]) == BUTTON_RELEASED)
                {
                    if (ReturnTime_100us() - press_time[i] >= BUTTON_SHORT_PRESS_TIME_100us)
                    {
                        if (spress_func[i])
                            spress_func[i]();
                    }
                    state[i] = BUTTON_RELEASED;
                }
                else if (ReturnTime_100us() - press_time[i] > BUTTON_LONG_PRESS_TIME_100us)
                {
                    if (lpress_func[i])
                        lpress_func[i]();
                    state[i] = BUTTON_LONGPRESS;
                }
            case BUTTON_LONGPRESS:
                if ((button_state)HAL_GPIO_ReadPin((GPIO_TypeDef *)button_gpio[i], button_pin[i]) == BUTTON_RELEASED)
                {
                    state[i] = BUTTON_RELEASED;
                }

                break;
            }
        }
    }
}

/**
 * @brief Returns the de-bounced logical state of the button
 *
 * @param btn button index
 * @return true if button is PRESSED
 * @return false if button is RELEASED
 */
bool button_get(button btn)
{
    return state[btn] != BUTTON_RELEASED && ReturnTime_100us() - press_time[btn] >= BUTTON_SHORT_PRESS_TIME_100us;
}

void button_set_shortpress(button btn, func_type function)
{
    spress_func[btn] = function;
}

void button_set_longpress(button btn, func_type function)
{
    lpress_func[btn] = function;
}
