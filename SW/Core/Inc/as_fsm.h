/**
 * @file as_fsm.h
 * @author Matteo Bonora (matteo.bonora@studenti.polito.it)
 * @brief Mirror the state of the Autonomous System to drive the ASSI lights and buzzer
 * @date 2022-05-26
 */
#pragma once

typedef enum
{
    AS_OFF = 1,
    AS_READY = 2,
    AS_DRIVING = 3,
    AS_EMERGENCY = 4,
    AS_FINISHED = 5,
    AS_TEST
} as_state_t;

// State keeper
extern as_state_t as_state;

/**
 * @brief Runs the AS finite state machine
 */
void as_run();