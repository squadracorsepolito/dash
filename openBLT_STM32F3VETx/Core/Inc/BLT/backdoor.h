/************************************************************************************//**
* \file         Source/backdoor.h
* \brief        Bootloader backdoor entry header file.
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
#ifndef BACKDOOR_H
#define BACKDOOR_H

/****************************************************************************************
* Function prototypes
****************************************************************************************/
void       BackDoorInit(void);
void       BackDoorCheck(void);
#if (BOOT_BACKDOOR_HOOKS_ENABLE == 0)
void       BackDoorSetExtension(blt_int32u extension_ms);
blt_int32u BackDoorGetExtension(void);
void       BackDoorRestartTimer(void);
#endif

/****************************************************************************************
* Macro definitions
****************************************************************************************/
#if (BOOT_BACKDOOR_HOOKS_ENABLE == 0)
#ifndef BOOT_BACKDOOR_ENTRY_TIMEOUT_MS
/** \brief Sets the time in milliseconds that the backdoor is open, but allow an
 *         override for this time. To change this value, simply add the macro
 *         BOOT_BACKDOOR_ENTRY_TIMEOUT_MS to blt_conf.h with your desired backdoor
 *         open time in milliseconds.
 */
#define BOOT_BACKDOOR_ENTRY_TIMEOUT_MS  (1000)
#endif
#endif /* BOOT_BACKDOOR_HOOKS_ENABLE == 0 */

#endif /* BACKDOOR_H */
/*********************************** end of backdoor.h *********************************/
