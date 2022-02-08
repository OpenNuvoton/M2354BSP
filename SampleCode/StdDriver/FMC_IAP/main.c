/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to call LDROM functions from APROM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


extern int32_t IAP_Func0(int32_t n);
extern int32_t IAP_Func1(int32_t n);
extern int32_t IAP_Func2(int32_t n);
extern int32_t IAP_Func3(int32_t n);

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
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
         This is a simple sample code for LDROM in new IAP mode.
         The base address is 0x100000.
         The base address for function table is defined by FUN_TBL_BASE.
     */

    printf("+---------------------------------------------------------+\n");
    printf("|          Flash Memory Controller use IAP function       |\n");
    printf("+---------------------------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable ISP function */
    FMC_Open();

    IAP_Func0(1);
    IAP_Func1(2);
    IAP_Func2(3);
    IAP_Func3(4);

    /* Disable ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nDone\n");
    while(SYS->PDID) __WFI();

}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


