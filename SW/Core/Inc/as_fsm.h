#pragma once

typedef enum
{
    AS_OFF = 0,
    AS_READY,
    AS_DRIVING,
    AS_FINISHED,
    AS_EMERGENCY,
    AS_TEST
} as_state_t;

extern as_state_t as_state;

void as_run();