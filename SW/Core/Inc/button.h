
#pragma once

#include <inttypes.h>
#include <stdbool.h>

#define BUTTON_COUNT 3
#define BUTTON_SAMPLE_TIME_100us 200

typedef enum
{
    TS_CK = 0,
    TS_EX = 1,
    Spare = 2
} button;

typedef enum
{
    BUTTON_PRESSED = 0,
    BUTTON_RELEASED = 1
} button_state;

void button_tick();
button_state button_get(button btn);
