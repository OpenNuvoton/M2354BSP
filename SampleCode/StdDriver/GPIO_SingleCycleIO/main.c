/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show GPIO single cycle IO bus performance.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"




void SYS_Init(void);
void UART0_Init(void);
void GPIO_SingleCycleIO_Test(void);


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

    /* Enable UART0, TIMER0 and TIMER2 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);

    /* Select UART0, TIMER0 and TIMER2 module clock source and clock divider */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set Timer 2 event counter pin */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~TM2_PD0_Msk)) | TM2_PD0;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|    GPIO Single Cycle IO Bus Sample Code    |\n");
    printf("+--------------------------------------------+\n\n");
    printf("GPIO supports Single Cycle IO Bus. User can access GPIO in one clock cycle.\n");
    printf("This sample code toggles PA.0 state 50 times and measures the toggle speed.\n");
    printf("Please connect PA.0 and PD.0(TIMER2 event counter pin).\n");
    printf("Enter any key to continue.\n\n");
    getchar();
    printf("Testing ...\n\n");

    /* Configure PA.0 as output mode */
    PA0 = 0;
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);

    /* Toggle PA.0 state 50 times and measure the toggle speed. */
    GPIO_SingleCycleIO_Test();

    while(1);

}
