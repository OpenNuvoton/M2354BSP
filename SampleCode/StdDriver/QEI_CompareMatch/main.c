/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of QEI compare function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define QEI0A   PC0
#define QEI0B   PC1


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void QEI0_IRQHandler(void);
void SYS_Init(void);
int32_t main(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  QEI0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void QEI0_IRQHandler(void)
{

    if(QEI_GET_INT_FLAG(QEI0, QEI_STATUS_CMPF_Msk))     /* Compare-match flag */
    {
        printf("Compare-match INT!\n\n");
        QEI_CLR_INT_FLAG(QEI0, QEI_STATUS_CMPF_Msk);
    }

    if(QEI_GET_INT_FLAG(QEI0, QEI_STATUS_OVUNF_Msk))    /* Counter Overflow or underflow flag */
    {
        printf("Overflow INT!\n\n");
        QEI_CLR_INT_FLAG(QEI0, QEI_STATUS_OVUNF_Msk);
    }
}

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

    /* Enable QEI0 module clock */
    CLK_EnableModuleClock(QEI0_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select TIMER0 module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* Set PH multi-function pins for QEI0_A, QEI0_B, QEI0_Z */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_QEI0_A | SYS_GPA_MFPL_PA3MFP_QEI0_B | SYS_GPA_MFPL_PA5MFP_QEI0_INDEX);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
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

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------------------+\n");
    printf("|          QEI Driver Sample Code      |\n");
    printf("+--------------------------------------+\n\n");
    printf("  >> Please connect PC.0 and PA.4 << \n");
    printf("  >> Please connect PC.1 and PA.3 << \n");
    printf("     Press any key to start test\n\n");
    getchar();

    /* Configure PA.0 and PA.1 as output mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT1, GPIO_MODE_OUTPUT);

    QEI0A = 0;
    QEI0B = 0;

    /* Set QEI counting mode as X4 Compare-counting mode,
       set maximum counter value and enable IDX, QEA and QEB input */
    QEI_Open(QEI0, QEI_CTL_X4_COMPARE_COUNTING_MODE, 0x20000);

    /* Set counter compare value */
    QEI_SET_CNT_CMP(QEI0, 0x10000);

    /* Enable compare function */
    QEI_ENABLE_CNT_CMP(QEI0);

    /* Enable QEI interrupt */
    QEI_EnableInt(QEI0, QEI_CTL_CMPIEN_Msk | QEI_CTL_OVUNIEN_Msk);

    /* Start QEI function */
    QEI_Start(QEI0);

    /* Wait compare-match and overflow interrupt happened */
    while(1)
    {
        QEI0A = 1;
        CLK_SysTickDelay(16);
        QEI0B = 1;
        CLK_SysTickDelay(16);
        QEI0A = 0;
        CLK_SysTickDelay(16);
        QEI0B = 0;
        CLK_SysTickDelay(16);
    }

}
