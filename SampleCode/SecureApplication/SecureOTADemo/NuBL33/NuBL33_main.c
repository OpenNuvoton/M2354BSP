/**************************************************************************//**
 * @file     NuBL32_main.c
 * @version  V1.00
 * @brief    Demonstrate NuBL33. (Non-secure code)
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include "NuBL_common.h"


/* Non-secure Callable function of NuBL32 */
extern void ShowCountersInNuBL32(uint32_t *in);
extern void BL32_OTA_Start(void);
extern void WdtResetCnt(void);
extern int32_t BL32_GetBL33FwVer(uint32_t * pu32FwVer);


void SysTick_Handler(void);

void SysTick_Handler(void)
{
    static uint32_t u32Ticks;

    if(u32Ticks > 800) /* 8s*/
    {
        WdtResetCnt();
        u32Ticks = 0;
    }
    u32Ticks++;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t    u32FwVer = 0;
    uint32_t u32Cfg = g_FWinfoInitial.mData.u32AuthCFGs;

    printf("\n");
    printf("+--------------------------------------------+\n");
    printf("|    M2354 NuBL33(Non-secure) Sample Code    |\n");
    printf("+--------------------------------------------+\n\n");
    printf("\n[AuthCFG: 0x%08x]\n", u32Cfg);

    if(BL32_GetBL33FwVer((uint32_t *)&u32FwVer) == 0)
        printf("NuBL33 Firmware Ver: 0x%08x\n\n", u32FwVer);
    else
        printf("NuBL33 Firmware Ver: N/A\n\n");

    /* Generate Systick interrupt each 10 ms */
    SystemCoreClockUpdate();

    SysTick_Config(SystemCoreClock / 100);

    BL32_OTA_Start();
//    ShowCountersInNuBL32(&temp);

    while(1) {}
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
