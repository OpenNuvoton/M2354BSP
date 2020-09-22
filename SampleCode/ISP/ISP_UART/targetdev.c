/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 * @version  0x32
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "isp_user.h"


// Supports 1M (APROM)
uint32_t GetApromSize()
{
    return 0x100000;
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
