/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Implement a multi-boot system to boot from different applications in APROM.
 *           A LDROM code and 4 APROM code are implemented in this sample code.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#if !defined(__ICCARM__) && !defined(__GNUC__)
extern uint32_t Image$$RO$$Base;
#endif

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
    uint8_t u8Ch;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0~0x1ff.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:

           For Keil/IAR project:
            FMC_MultiBoot, RO=0x0
            FMC_Boot0,  RO=0x4000
            FMC_Boot1,  RO=0x8000
            FMC_Boot2,  RO=0xC000
            FMC_Boot3,  RO=0x10000
            FMC_BootLD, RO=0x100000

           For GCC project:
            FMC_MultiBoot, RO=0x0
            FMC_Boot1,  RO=0x8000
            FMC_Boot3,  RO=0x10000

        2. Reset MCU to execute FMC_MultiBoot.

    */

    printf("\n\n");
    printf("+----------------------------------------------+\n");
    printf("|     Multi-Boot Sample Code(0x%08X)       |\n", FMC_GetVECMAP());
    printf("+----------------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

#if defined(__ICCARM__) || defined(__GNUC__)
    printf("VECMAP = 0x%x\n", FMC_GetVECMAP());
#else
#ifndef BootLD
    printf("Current RO Base = 0x%x, VECMAP = 0x%x\n", (uint32_t)&Image$$RO$$Base, FMC_GetVECMAP());
#else
    printf("VECMAP = 0x%x\n", FMC_GetVECMAP());
#endif
#endif

    printf("Select one boot image: \n");
#if defined(__ARMCC_VERSION) || defined(__ICCARM__)
    printf("[0] Boot 0,  base = 0x4000\n");
    printf("[1] Boot 1,  base = 0x8000\n");
    printf("[2] Boot 2,  base = 0xC000\n");
    printf("[3] Boot 3,  base = 0x10000\n");
    printf("[4] Boot LD, base = 0x100000\n");
#else
    printf("[1] Boot 1, base = 0x8000\n");
    printf("[3] Boot 3, base = 0x10000\n");
#endif
    printf("[Others] Boot, base = 0x0\n");

    u8Ch = (uint8_t)getchar();
    switch(u8Ch)
    {
#if defined(__ARMCC_VERSION) || defined(__ICCARM__)
        case '0':
            FMC_SetVectorPageAddr(0x4000);
            break;
        case '1':
            FMC_SetVectorPageAddr(0x8000);
            break;
        case '2':
            FMC_SetVectorPageAddr(0xC000);
            break;
        case '3':
            FMC_SetVectorPageAddr(0x10000);
            break;
        case '4':
            FMC_SetVectorPageAddr(0x100000);
            break;
#else
    case '1':
        FMC_SetVectorPageAddr(0x8000);
        break;
    case '3':
        FMC_SetVectorPageAddr(0x10000);
        break;
#endif
        default:
            FMC_SetVectorPageAddr(0x0);
            break;
    }

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_SetVectorPageAddr failed!\n");
        goto lexit;
    }

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

lexit:

    while(1);

}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


