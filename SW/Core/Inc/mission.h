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

#define NUM_MISSIONS 8

typedef enum
{
    MISSION_ACCEL = 1,
    MISSION_SKIDPAD = 2,
    MISSION_AUTOX = 3,
    MISSION_TRACKDRIVE = 4,
    MISSION_EBSTEST = 5,
    MISSION_INSPECT = 6,
    MISSION_MANUAL = 7,
    MISSION_NO = 0,
    // MISSION_CAN_DSPACE_DO_THIS = 8,

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
 * @brief If the mission is not confirmed, set the given mission
 *
 * @param mission New mission
 */
void mission_set(mission_t mission);

/**
 * @brief Sets the AMI according to current mission and confirmation
 */
void mission_run();