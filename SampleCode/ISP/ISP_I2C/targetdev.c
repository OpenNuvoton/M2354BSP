/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 * @version  0x32
 * @date     24, August, 2020
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "isp_user.h"

uint32_t GetApromSize(void)
{
    return 0x100000;
}

/* Data Flash is shared with APROM. */
/* The size and start address are defined in CONFIG1.*/
void GetDataFlashInfo(uint32_t *u32Addr, uint32_t *u32Size)
{
    uint32_t uData;

    *u32Size = 0;
    FMC_ReadUser(CONFIG0, &uData);

    /* DFEN enable */
    if((uData & 0x01) == 0)
    {
        FMC_ReadUser(CONFIG1, &uData);

        /* Filter the reserved bits in CONFIG1 */
        uData &= 0x000FFFFF;

        /* avoid CONFIG1 value from error */
        if(uData > g_u32ApromSize || uData & (FMC_FLASH_PAGE_SIZE - 1))
        {
            uData = g_u32ApromSize;
        }

        *u32Addr = uData;
        *u32Size = g_u32ApromSize - uData;
    }
    else
    {
        *u32Addr = g_u32ApromSize;
        *u32Size = 0;
    }
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
