
#pragma once

#include <inttypes.h>
#include <stdbool.h>

#define BUTTON_COUNT 3
#define BUTTON_SAMPLE_TIME_100us 100
#define BUTTON_DEBOUNCE_TIME_100us (BUTTON_SAMPLE_TIME_100us * 10)

typedef enum
{
    BUTTON_TS_CK = 0,
    BUTTON_TS_EX = 1,
    BUTTON_Spare = 2
} button;

typedef enum
{
    BUTTON_PRESSED = 0,
    BUTTON_RELEASED = 1
} button_state;

void button_sample();
bool button_get(button btn);
