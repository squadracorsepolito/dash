#pragma once

#include "main.h"

extern uint32_t counter;

void LedBlinking(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t period);
int delay_fun(uint32_t *delay_100us_last, uint32_t delay_100us);
uint32_t ReturnTime_100us(void);