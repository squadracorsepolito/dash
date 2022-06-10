#include "can_watchdog.h"
#include "utils.h"

uint32_t wdg_timestamps_100us[WDG_NUM_BOARDS];
// TODO: define timeouts
uint32_t wdg_timeouts_100us[WDG_NUM_BOARDS] = {
    [WDG_DSPACE] = 5000,
    [WDG_TLB] = 5000,
    [WDG_SENSORS] = 5000};

void wdg_reset(wdg_boards board, uint32_t timestamp_100us)
{
    wdg_timestamps_100us[board] = timestamp_100us;
}

uint8_t wdg_check()
{
    uint8_t boards = 0;

    for (uint8_t board = 0; board < WDG_NUM_BOARDS; board++)
    {
        if (ReturnTime_100us() - wdg_timestamps_100us[board] > wdg_timeouts_100us[board])
        {
            boards |= 1 << board;
        }
    }

    return boards;
}