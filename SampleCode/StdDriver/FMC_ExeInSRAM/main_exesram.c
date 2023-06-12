/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Implement a code and execute in SRAM to program embedded Flash.
 *           (Support IAR/GCC)
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define SRAM_CODE_EXE_ADDR  0x20010000
#define SRAM_CODE_BASE      0x8000
#define SRAM_CODE_SIZE      0x2000

typedef void (FUNC_PTR)(void);

extern int32_t FlashAccess_OnSRAM(void);

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

    FUNC_PTR    *func;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      FMC Write/Read code execute in SRAM Sample Code      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
       This sample code is used to demonstrate how to implement a code to execute in SRAM.
    */

#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))

    printf("Will branch to address: 0x%x\n", (uint32_t)FlashAccess_OnSRAM);

    if(FlashAccess_OnSRAM())
    {
        printf("Flash access return error.\n");
    }
    else
    {
        printf("Flash access return ok.\n");
    }

#else

    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("Execute demo code in APROM ==>\n");

    FlashAccess_OnSRAM();

    memcpy((uint8_t *)SRAM_CODE_EXE_ADDR, (uint8_t *)SRAM_CODE_BASE, SRAM_CODE_SIZE);

    printf("Execute demo code in SRAM ==>\n");

    func = (FUNC_PTR *)(SRAM_CODE_EXE_ADDR + 1);

    func();   /* branch to exeinsram.o in SRAM  */

    printf("Execute demo code in SRAM Done!\n");
    /* Lock protected registers */
    SYS_LockReg();

#endif

    printf("\nFMC Sample Code Completed.\n");

    while(1);

}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


