/***************************************************************************//**
 * @file     fmc_user.c
 * @brief    M2351 series FMC driver source file
 * @version  2.0.0
 * @date     24, August, 2020
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "fmc_user.h"

#define FMC_BLOCK_SIZE           (FMC_FLASH_PAGE_SIZE * 4UL)

int FMC_Proc(unsigned int u32Cmd, unsigned int u32AddrStart, unsigned int u32AddrEnd, unsigned int *pu32Data);

int FMC_Proc(unsigned int u32Cmd, unsigned int u32AddrStart, unsigned int u32AddrEnd, unsigned int *pu32Data)
{
    unsigned int u32Addr, u32Reg;

    for(u32Addr = u32AddrStart; u32Addr < u32AddrEnd; pu32Data++)
    {
        FMC->ISPCMD = u32Cmd;
        FMC->ISPADDR = u32Addr;

        if(u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPDAT = *pu32Data;
        }

        FMC->ISPTRG = 0x1;
        __ISB()

        /* Wait ISP cmd complete */
        while(FMC->ISPTRG);

        u32Reg = FMC->ISPCTL;

        if(u32Reg & FMC_ISPCTL_ISPFF_Msk)
        {
            FMC->ISPCTL = u32Reg;
            return -1;
        }

        if(u32Cmd == FMC_ISPCMD_READ)
        {
            *pu32Data = FMC->ISPDAT;
        }

        if(u32Cmd == FMC_ISPCMD_PAGE_ERASE)
        {
            u32Addr += FMC_FLASH_PAGE_SIZE;
        }
        else
        {
            u32Addr += 4;
        }
    }

    return 0;
}

/**
 * @brief      Program 32-bit data into specified address of flash
 *
 * @param[in]  u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32data  32-bit Data to program
 *
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of TRM.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function. User can check the status of
 *             Register Write-Protection Function with DrvSYS_IsProtectedRegLocked().
 */
int FMC_WriteUser(unsigned int u32Addr, unsigned int u32Data)
{
    return FMC_Proc(FMC_ISPCMD_PROGRAM, u32Addr, u32Addr + 4, &u32Data);
}

/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The data of specified address
 *
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 * @note
 *              Please make sure that Register Write-Protection Function has been disabled
 *              before using this function. User can check the status of
 *              Register Write-Protection Function with DrvSYS_IsProtectedRegLocked().
 */
int FMC_ReadUser(unsigned int u32Addr, unsigned int *pu32Data)
{
    return FMC_Proc(FMC_ISPCMD_READ, u32Addr, u32Addr + 4, pu32Data);
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 512 bytes.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function. User can check the status of
 *             Register Write-Protection Function with DrvSYS_IsProtectedRegLocked().
 */
int FMC_EraseUser(unsigned int u32Addr)
{
    return FMC_Proc(FMC_ISPCMD_PAGE_ERASE, u32Addr, u32Addr + 4, 0);
}

void ReadData(unsigned int u32AddrStart, unsigned int u32AddrEnd, unsigned int *pu32Data)    /* Read data from flash */
{
    FMC_Proc(FMC_ISPCMD_READ, u32AddrStart, u32AddrEnd, pu32Data);
    return;
}

void WriteData(unsigned int u32AddrStart, unsigned int u32AddrEnd, unsigned int *pu32Data)  /* Write data into flash */
{
    FMC_Proc(FMC_ISPCMD_PROGRAM, u32AddrStart, u32AddrEnd, pu32Data);
    return;
}



int EraseAP(unsigned int u32AddrStart, unsigned int u32TotalSize)
{
    unsigned int u32Addr, u32Cmd, u32UintSize;
    u32Addr = u32AddrStart;

    while(u32TotalSize > 0)
    {
        if((u32TotalSize >= FMC_BANK_SIZE) && !(u32Addr & (FMC_BANK_SIZE - 1)))
        {
            u32Cmd = FMC_ISPCMD_BANK_ERASE;
            u32UintSize = FMC_BANK_SIZE;
        }
        else
        {
            u32Cmd = FMC_ISPCMD_PAGE_ERASE;
            u32UintSize = FMC_FLASH_PAGE_SIZE;
        }

        FMC->ISPCMD = u32Cmd;
        FMC->ISPADDR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB()

        /* Wait for ISP command done. */
        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

        if(FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
        {
            FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
            return -1;
        }

        u32Addr += u32UintSize;
        u32TotalSize -= u32UintSize;
    }

    return 0;
}

void UpdateConfig(unsigned int *pu32Data, unsigned int *pu32Res)
{
    FMC_ENABLE_CFG_UPDATE();
    FMC_Proc(FMC_ISPCMD_PAGE_ERASE, CONFIG0, CONFIG0 + 16, 0);
    FMC_Proc(FMC_ISPCMD_PROGRAM, CONFIG0, CONFIG0 + 16, pu32Data);

    if(pu32Res)
    {
        FMC_Proc(FMC_ISPCMD_READ, CONFIG0, CONFIG0 + 16, pu32Res);
    }

    FMC_DISABLE_CFG_UPDATE();
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
