/**
 * @file utils.h
 * @author Matteo Bonora (matteo.bonora@studenti.polito.it)
 * @brief Small collection of useful functions
 * @date 2022-05-26
 */

#pragma once

#include "main.h"
#include <stdbool.h>

/**
 * @brief Toggles an output at the given frequency
 *
 * @param GPIOx
 * @param GPIO_Pin
 * @param last Last output toggle time
 * @param period Frequency period
 */
void LedBlinking(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t *last, uint32_t period);

/**
 * @brief Software non-blocking delay function
 *
 * @param delay_100us_last Last delay timestamp
 * @param delay_100us Desired delay
 * @return true
 * @return false
 */
bool delay_fun(uint32_t *delay_100us_last, uint32_t delay_100us);

uint32_t ReturnTime_100us(void);
