#pragma once

#include "main.h"
#include <inttypes.h>

typedef enum
{
    MISSION_NO = 0,
    MISSION_ACCEL = 1,
    MISSION_SKIDPAD = 2,
    MISSION_AUTOX = 3,
    MISSION_TRACKDRI = 4,
    MISSION_EBSTEST = 5,
    MISSION_INSPECT = 6,
    MISSION_MANUAL = 7,
} mission_t;

mission_t ami_get();

void ami_set(mission_t mission);