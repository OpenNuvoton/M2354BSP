/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how mbedTLS ECDH function works.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define MBEDTLS_EXIT_SUCCESS    0
#define MBEDTLS_EXIT_FAILURE    -1


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

extern int ECDHTest(void);

int32_t main(void)
{
    int32_t  i32Ret = MBEDTLS_EXIT_SUCCESS;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code is used to show how to use StdDriver API to implement ISP functions.
    */

    printf("\n\n");
    printf("+---------------------------------+\n");
    printf("|        ECDH Sample Code         |\n");
    printf("+---------------------------------+\n");


    printf("\n ECDH test start...\n\n");
    i32Ret = ECDHTest();
    printf("\n ECDH test done ...\n");


    if(i32Ret == MBEDTLS_EXIT_SUCCESS)
    {
        printf("\n Test OK\n");
    }
    else
    {
        printf("\n Test fail\n");
    }

    while(1);

}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


