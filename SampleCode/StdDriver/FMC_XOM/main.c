/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use FMC ISP APIs config/erase XOM region.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define XOMR0_Base    0x10000

extern int32_t LibXOMAdd(uint32_t a, uint32_t b);

void SYS_Init(void);

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

}

int32_t main(void)
{
    int32_t i32Status;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code is used to show how to use StdDriver API to enable/erase XOM.
    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|  FMC XOM config & erase  Sample Code   |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function and enable APROM active */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_USER_CONFIG_0));

    printf("XOM Status = 0x%X\n", FMC->XOMSTS);
    printf("Any key to continue...\n");
    getchar();

    if((FMC->XOMSTS & 0x1) != 0x1)
    {
        /* Config XOMR0 */
        if(FMC_GetXOMState(XOMR0) == 0)
        {
            i32Status = FMC_ConfigXOM(XOMR0, XOMR0_Base, 1);
            if(i32Status)
                printf("XOMR0 Config fail...\n");
            else
                printf("XOMR0 Config OK...\n");
        }

        printf("\nAny key to reset chip to enable XOM regions...\n");
        getchar();

        /* Reset chip to enable XOM region. */
        SYS_ResetChip();
        while(1) {}
    }

    /* Run XOM function */
    printf("\n");
    printf(" 100 + 200 = %d\n", LibXOMAdd(100, 200));

    printf("\nXOMR0 active success....\n");
    printf("\nAny key to Erase XOM...\n");
    getchar();

    if((FMC->XOMSTS & 0x1) == 0x1)
    {
        /* Erase XOMR0 region */
        if(FMC_EraseXOM(XOMR0) == 0)
            printf("Erase XOMR0....OK\n");
        else
            printf("Erase XOMR0....Fail\n");

        printf("\nAny key to reset chip...\n");
        getchar();

        /* Reset chip to finish erase XOM region. */
        SYS_ResetChip();
    }

    while(1);
}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


