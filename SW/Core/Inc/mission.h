/**
 * @file mission.h
 * @author Matteo Bonora (matteo.bonora@studenti.polito.it)
 * @brief Handles the autonomous mission selection and the AMI lights.
 * @date 2022-05-08
 */

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

/**
 * @brief Setups callback functions for the mission select button
 */
void mission_setup();

/**
 * @return true mission has been confirmed
 * @return false mission is not confirmed
 */
bool mission_is_confirmed();
/**
 * @return mission_t the currently selected mission
 */
mission_t mission_get();

/**
 * @brief If the mission is not confirmed, select a new mission
 *
 * @param mission New mission
 */
void mission_set(mission_t mission);

/**
 * @brief Sets the AMI according to current mission and confirmation
 */
void mission_run();