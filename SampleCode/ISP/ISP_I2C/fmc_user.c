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

int FMC_Proc(uint32_t u32Cmd, uint32_t u32AddrStart, uint32_t u32AddrEnd, uint32_t *pu32Data);

int FMC_Proc(uint32_t u32Cmd, uint32_t u32AddrStart, uint32_t u32AddrEnd, uint32_t *pu32Data)
{
    unsigned int u32Addr, u32Reg;
    uint32_t u32TimeOutCnt;

    for(u32Addr = u32AddrStart; u32Addr < u32AddrEnd; pu32Data++)
    {
        FMC->ISPCMD = u32Cmd;
        FMC->ISPADDR = u32Addr;

        if(u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPDAT = *pu32Data;
        }

        FMC->ISPTRG = 0x1;
        __ISB();

        /* Wait ISP cmd complete */
        u32TimeOutCnt = FMC_TIMEOUT_WRITE;
        while(FMC->ISPTRG)
        {
            if(--u32TimeOutCnt == 0)
                return -1;
        }

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
int FMC_WriteUser(uint32_t u32Addr, uint32_t u32Data)
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
int FMC_ReadUser(uint32_t u32Addr, uint32_t *pu32Data)
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
int FMC_EraseUser(uint32_t u32Addr)
{
    return FMC_Proc(FMC_ISPCMD_PAGE_ERASE, u32Addr, u32Addr + 4, 0);
}

void ReadData(uint32_t u32AddrStart, uint32_t u32AddrEnd, uint32_t *pu32Data)    /* Read data from flash */
{
    FMC_Proc(FMC_ISPCMD_READ, u32AddrStart, u32AddrEnd, pu32Data);
    return;
}

void WriteData(uint32_t u32AddrStart, uint32_t u32AddrEnd, uint32_t *pu32Data)  /* Write data into flash */
{
    FMC_Proc(FMC_ISPCMD_PROGRAM, u32AddrStart, u32AddrEnd, pu32Data);
    return;
}



int EraseAP(uint32_t u32AddrStart, uint32_t u32TotalSize)
{
    unsigned int u32Addr, u32Cmd, u32UintSize;
    uint32_t u32TimeOutCnt;

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
        __ISB();

        /* Wait for ISP command done. */
        u32TimeOutCnt = FMC_TIMEOUT_ERASE;
        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if(--u32TimeOutCnt == 0)
                return -1;
        }

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

void UpdateConfig(uint32_t *pu32Data, uint32_t *pu32Res)
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
