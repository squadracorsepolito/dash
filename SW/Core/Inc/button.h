
#pragma once

#include <inttypes.h>
#include <stdbool.h>

#define BUTTON_SAMPLE_TIME_100us 300
#define BUTTON_SHORT_PRESS_TIME_100us 1000
#define BUTTON_LONG_PRESS_TIME_100us 8000

typedef enum
{
    BUTTON_TS_CK = 0,
    BUTTON_TS_EX,
    BUTTON_MISSION,
    BUTTON_COUNT
} button;

typedef enum
{
    BUTTON_PRESSED = 0,
    BUTTON_RELEASED = 1,
    BUTTON_LONGPRESS = 2
} button_state;

typedef void (*func_type)(void);

void button_sample();
bool button_get(button btn);
void button_set_shortpress(button btn, func_type function);
void button_set_longpress(button btn, func_type function);
uint32_t button_last_change_time(button btn);
