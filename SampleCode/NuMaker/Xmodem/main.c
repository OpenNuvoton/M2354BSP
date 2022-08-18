/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to transfer data through UART by Xmodem.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "xmodem.h"

void SYS_Init(void);

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC, HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC, HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

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
    int32_t i32Err;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);


    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|         Xmodem Sample Code             |\n");
    printf("+----------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

    printf("Waiting for Xmodem data transfer ...\n");

    /* Waiting for debug message print out */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk) == 0)
        if(--u32TimeOutCnt == 0) break;

    i32Err = Xmodem(0x80000);

    if(i32Err < 0)
    {
        printf("Xmodem transfer fail!\n");
    }
    else
    {
        printf("Xomdem transfer done!\n");
        printf("Total trnasfer size is %d\n", i32Err);

        printf("press any key to read back the transfer file.\n");
        getchar();
        printf("Waiting for receiver ...\n");


        i32Err = XmodemSend((uint8_t *)0x80000, i32Err);

        if(i32Err < 0)
        {
            printf("Xmodem transfer fail!\n");
        }
        else
        {
            printf("Transfer done.\nTotal trasnfer size is %d\n", i32Err);

        }

    }

    while(1);

}

