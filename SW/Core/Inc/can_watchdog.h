#pragma once
#include <inttypes.h>

typedef enum
{
    WDG_DSPACE,
    WDG_TLB,
    WDG_SENSORS,
    WDG_NUM_BOARDS
} wdg_boards;

extern uint32_t wdg_timestamps_100us[WDG_NUM_BOARDS];

void wdg_reset(wdg_boards board, uint32_t timestamp_100us);
uint8_t wdg_check();
