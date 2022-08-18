/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement timer counting in periodic mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_au32TMRINTCount[6] = {0};

void TMR0_IRQHandler(void);
void TMR1_IRQHandler(void);
void TMR2_IRQHandler(void);
void TMR3_IRQHandler(void);
void TMR4_IRQHandler(void);
void TMR5_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);


/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_M2354.s.
 */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        g_au32TMRINTCount[0]++;
    }
}

/**
 * @brief       Timer1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer1 default IRQ, declared in startup_M2354.s.
 */
void TMR1_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);

        g_au32TMRINTCount[1]++;
    }
}

/**
 * @brief       Timer2 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer2 default IRQ, declared in startup_M2354.s.
 */
void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);

        g_au32TMRINTCount[2]++;
    }
}

/**
 * @brief       Timer3 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer3 default IRQ, declared in startup_M2354.s.
 */
void TMR3_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        /* Clear Timer3 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);

        g_au32TMRINTCount[3]++;
    }
}

/**
 * @brief       Timer4 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer4 default IRQ, declared in startup_M2354.s.
 */
void TMR4_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER4) == 1)
    {
        /* Clear Timer4 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER4);

        g_au32TMRINTCount[4]++;
    }
}

/**
 * @brief       Timer5 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer5 default IRQ, declared in startup_M2354.s.
 */
void TMR5_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER5) == 1)
    {
        /* Clear Timer5 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER5);

        g_au32TMRINTCount[5]++;
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_EnableModuleClock(TMR4_MODULE);
    CLK_EnableModuleClock(TMR5_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HIRC, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HXT, 0);
    CLK_SetModuleClock(TMR4_MODULE, CLK_CLKSEL3_TMR4SEL_MIRC, 0);
    CLK_SetModuleClock(TMR5_MODULE, CLK_CLKSEL3_TMR5SEL_HXT, 0);
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32InitCount, au32Counts[6];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|    Timer Periodic Interrupt Sample Code    |\n");
    printf("+--------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HXT       \n");
    printf("    - Time-out frequency is 1 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer1 Settings:\n");
    printf("    - Clock source is HCLK      \n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer2 Settings:\n");
    printf("    - Clock source is HIRC      \n");
    printf("    - Time-out frequency is 4 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HXT       \n");
    printf("    - Time-out frequency is 8 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer4 Settings:\n");
    printf("    - Clock source is MIRC      \n");
    printf("    - Time-out frequency is 10 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer5 Settings:\n");
    printf("    - Clock source is HXT       \n");
    printf("    - Time-out frequency is 12 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Check Timer0 ~ Timer5 interrupt counts are reasonable or not.\n\n");

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER0);

    /* Open Timer1 in periodic mode, enable interrupt and 2 interrupt ticks per second */
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 2);
    TIMER_EnableInt(TIMER1);

    /* Open Timer2 in periodic mode, enable interrupt and 4 interrupt ticks per second */
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 4);
    TIMER_EnableInt(TIMER2);

    /* Open Timer3 in periodic mode, enable interrupt and 8 interrupt ticks per second */
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 8);
    TIMER_EnableInt(TIMER3);

    /* Open Timer4 in periodic mode, enable interrupt and 10 interrupt ticks per second */
    TIMER_Open(TIMER4, TIMER_PERIODIC_MODE, 10);
    TIMER_EnableInt(TIMER4);

    /* Open Timer5 in periodic mode, enable interrupt and 12 interrupt ticks per second */
    TIMER_Open(TIMER5, TIMER_PERIODIC_MODE, 12);
    TIMER_EnableInt(TIMER5);

    /* Enable Timer0 ~ Timer5 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
    NVIC_EnableIRQ(TMR1_IRQn);
    NVIC_EnableIRQ(TMR2_IRQn);
    NVIC_EnableIRQ(TMR3_IRQn);
    NVIC_EnableIRQ(TMR4_IRQn);
    NVIC_EnableIRQ(TMR5_IRQn);

    /* Clear Timer0 ~ Timer5 interrupt counts to 0 */
    g_au32TMRINTCount[0] = g_au32TMRINTCount[1] = g_au32TMRINTCount[2] = g_au32TMRINTCount[3] = 0;
    g_au32TMRINTCount[4] = g_au32TMRINTCount[5] = 0;
    u32InitCount = g_au32TMRINTCount[0];

    /* Start Timer0 ~ Timer5 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER1);
    TIMER_Start(TIMER2);
    TIMER_Start(TIMER3);
    TIMER_Start(TIMER4);
    TIMER_Start(TIMER5);

    /* Check Timer0 ~ Timer5 interrupt counts */
    printf("# Timer interrupt counts :\n");
    while(u32InitCount < 20)
    {
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
            au32Counts[0] = g_au32TMRINTCount[0];
            au32Counts[1] = g_au32TMRINTCount[1];
            au32Counts[2] = g_au32TMRINTCount[2];
            au32Counts[3] = g_au32TMRINTCount[3];
            au32Counts[4] = g_au32TMRINTCount[4];
            au32Counts[5] = g_au32TMRINTCount[5];
            printf("    TMR0:%3d    TMR1:%3d    TMR2:%3d    TMR3:%3d    TMR4:%3d    TMR5:%3d \n",
                   au32Counts[0], au32Counts[1], au32Counts[2], au32Counts[3], au32Counts[4], au32Counts[5]);
            u32InitCount = g_au32TMRINTCount[0];

            if((au32Counts[1] > (au32Counts[0] * 2 + 1)) || (au32Counts[1] < (au32Counts[0] * 2 - 1)) ||
                    (au32Counts[2] > (au32Counts[0] * 4 + 1)) || (au32Counts[2] < (au32Counts[0] * 4 - 1)) ||
                    (au32Counts[3] > (au32Counts[0] * 8 + 1)) || (au32Counts[3] < (au32Counts[0] * 8 - 1)) ||
                    (au32Counts[4] > (au32Counts[0] * 10 + 1)) || (au32Counts[4] < (au32Counts[0] * 10 - 1)) ||
                    (au32Counts[5] > (au32Counts[0] * 12 + 1)) || (au32Counts[5] < (au32Counts[0] * 12 - 1)))
            {
                printf("*** FAIL ***\n");
                goto lexit;
            }
        }
    }

    printf("*** PASS ***\n");

lexit:

    while(1) {}
}
