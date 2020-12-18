/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to reload the EWWDT counter value.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8EWWDTINTCounts;

void EWWDT_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);


/**
 * @brief       IRQ Handler for EWWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The EWWDT_IRQHandler is default IRQ of EWWDT, declared in startup_M2354.s.
 */
void EWWDT_IRQHandler(void)
{
    if(EWWDT_GET_INT_FLAG() == 1)
    {
        /* Clear EWWDT compare match interrupt flag */
        EWWDT_CLEAR_INT_FLAG();

        PA2 ^= 1;

        g_u8EWWDTINTCounts++;

        if(g_u8EWWDTINTCounts < 10)
        {
            /* To reload the EWWDT counter value to 0x3F */
            EWWDT_RELOAD_COUNTER();
        }

        printf("EWWDT compare match interrupt occurred. (%d)\n", g_u8EWWDTINTCounts);
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
    /* Enable EWWDT module clock */
    CLK_EnableModuleClock(EWWDT_MODULE);
    CLK_SetModuleClock(EWWDT_MODULE, CLK_CLKSEL1_EWWDTSEL_HCLK_DIV2048, 0);
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
    double dPeriodTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------+\n");
    printf("|    EWWDT Compare Match Interrupt Sample Code    |\n");
    printf("+-------------------------------------------------+\n\n");

    /* To check if system has been reset by EWWDT time-out reset or not */
    if(EWWDT_GET_RESET_FLAG() == 1)
    {
        printf("*** System has been reset by EWWDT time-out reset event. [EWWDT_CTL: 0x%08X] ***\n\n", EWWDT->CTL);
        EWWDT_CLEAR_RESET_FLAG();
        while(1) {}
    }

    dPeriodTime = (((double)(1000000 * 2048) / (double)SystemCoreClock) * 1024) * 32;

    printf("# EWWDT Settings: \n");
    printf("    - Clock source is PCLK0/2048 (%d Hz)    \n", (SystemCoreClock / 2048));
    printf("    - EWWDT counter prescale period is 1024, \n");
    printf("        and max EWWDT time-out period is 1024 * (64 * EWWDT_CLK)\n");
    printf("    - Interrupt enable                      \n");
    printf("    - Window Compare value is 32            \n");
    printf("# System will generate first EWWDT compare match interrupt event after %.2f ms.\n", (dPeriodTime / 1000));
    printf("    1.) use PA.2 high/low period to check EWWDT compare match interrupt period time\n");
    printf("    2.) reload EWWDT counter value to avoid EWWDT time-out reset system occurred\n");
    printf("        when interrupt counts less than 11.\n");
    printf("    3.) do not reload EWWDT counter value to generate EWWDT time-out reset system event\n");
    printf("        when interrupt counts large than 10.\n\n");

    /* Use PA.2 to check EWWDT compare match interrupt period time */
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE2_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE2_Pos);
    PA2 = 1;

    /* Enable EWWDT NVIC */
    NVIC_EnableIRQ(EWWDT_IRQn);

    g_u8EWWDTINTCounts = 0;

    /*
        Max time-out period is 1024*(64*EWWDT_CLK);
        EWWDT compare value is 32;
        Enable EWWDT compare match interrupt;
    */
    /* Note: EWWDT_CTL register can be written only once after chip is powered on or reset */
    EWWDT_Open(EWWDT_PRESCALER_1024, 32, TRUE);

    printf("[EWWDT_CTL: 0x%08X]\n\n", EWWDT->CTL);

    while(1) {}
}
