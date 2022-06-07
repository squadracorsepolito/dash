#pragma once

#include "main.h"
#include <inttypes.h>
#include <stdbool.h>

typedef enum
{
    MISSION_ACCEL = 0,
    MISSION_SKIDPAD = 1,
    MISSION_AUTOX = 2,
    MISSION_TRACKDRIVE = 3,
    MISSION_EBSTEST = 4,
    MISSION_INSPECT = 5,
    MISSION_MANUAL = 6,
    MISSION_NO = 7,

    NUM_MISSIONS
} mission_t;

extern bool ami_selected;

void ami_setup();
bool ami_is_selected();
mission_t ami_get();
void ami_set_mission(mission_t mission);
void ami_run();