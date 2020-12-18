/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 19/11/22 2:06p $
 * @brief    Show how mbedTLS RSA function works.
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "rsa.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

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




extern int PEMtoRSA(void);
extern int RSAEncryptWithHashTest(int verbose);

int32_t main(void)
{
    uint32_t  u32Verbose;
    int32_t  i32Ret;

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
    printf("|         RSA Sample Code         |\n");
    printf("+---------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    i32Ret = MBEDTLS_EXIT_SUCCESS;

    printf("\n  PEM to RSA key test start\n");
    i32Ret = PEMtoRSA();
    printf("\n  PEM to RSA key test done\n");

    if(i32Ret != MBEDTLS_EXIT_SUCCESS)
    {
        printf("\n  Test fail\n");
        while(1);
    }
#if 0
    printf("\n RSA encrypt with hash test. \n Please enter the [verbose] value, then press Enter Key:\n");
    scanf("%d", &u32Verbose);
    printf("\n RSA encrypt with hash test start...   verbose[%d]\n", u32Verbose);
#else
    u32Verbose = 1;
    printf("\n  RSA encrypt with hash test start\n");
#endif

    i32Ret = RSAEncryptWithHashTest(u32Verbose);
    printf("\n  RSA encrypt with hash test done\n");

    if(i32Ret == MBEDTLS_EXIT_SUCCESS)
    {
        printf("\n  Test OK\n");
    }
    else
    {
        printf("\n  Test fail\n");
    }

    while(1);

}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


