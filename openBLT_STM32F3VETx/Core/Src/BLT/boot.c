/************************************************************************************//**
* \file         Source/boot.c
* \brief        Bootloader core module source file.
* \ingroup      Core
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2011  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "BLT/boot.h"                                /* bootloader generic header          */


/************************************************************************************//**
** \brief     Initializes the bootloader core.
** \return    none
**
****************************************************************************************/
void BootInit(void)
{
  /* initialize the CPU */
  CpuInit();
  /* initialize the watchdog */
  CopInit();
  /* initialize the millisecond timer */
  TimerInit();
  /* initialize the non-volatile memory driver */
  NvmInit();
  /* initialize the leds */
  LedBlinkInit(TimerGet());
#if (BOOT_COM_ENABLE > 0)
  /* initialize the communication module */
  ComInit();
#endif
  /* initialize the backdoor entry */
  BackDoorInit();
} /*** end of BootInit ***/


/************************************************************************************//**
** \brief     Task function of the bootloader core that drives the program.
** \return    none
**
****************************************************************************************/
void BootTask(void)
{
  /* service the watchdog */
  CopService();
  /* update the millisecond timer */
  TimerUpdate();
  #if (BOOT_COM_ENABLE > 0)
  /* process possibly pending communication data */
  ComTask();
  #endif
  /* control the backdoor */
  BackDoorCheck();
  /* Let LED blink if time has passed */
  LedBlinkTask();
} /*** end of BootTask ***/

// ONLY FOR F3
void CpuIrqDisable(void)
{
  __asm volatile ("cpsid i");
} /*** end of CpuIrqDisable ***/


/************************************************************************************//**
** \brief     Enable global interrupts.
** \return    none.
**
****************************************************************************************/
void CpuIrqEnable(void)
{
  __asm volatile ("cpsie i");
} /*** end of CpuIrqEnable ***/


/*********************************** end of boot.c *************************************/
