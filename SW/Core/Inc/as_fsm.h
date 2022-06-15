/**
 * @file as_fsm.h
 * @author Matteo Bonora (matteo.bonora@studenti.polito.it)
 * @brief Mirror the state of the Autonomous System to drive the ASSI lights and buzzer
 * @date 2022-05-26
 */
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