#pragma once

typedef enum
{
    AS_OFF = 0,
    AS_READY,
    AS_DRIVING,
    AS_FINISHED,
    AS_EMERGENCY
} as_state_t;

extern as_state_t state;

void as_run();