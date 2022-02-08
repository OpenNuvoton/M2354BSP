/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate Timer4, Timer5 PWM output waveform in power-down and wake-up functions.
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

extern int IsDebugFifoEmpty(void);
void TMR4_IRQHandler(void);
void TMR5_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);

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
    if(TPWM_GET_PERIOD_INT_FLAG(TIMER4) == 1)
    {
        /* Clear Timer4 PWM period interrupt flag */
        TPWM_CLEAR_PERIOD_INT_FLAG(TIMER4);

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
    if(TPWM_GET_CMP_UP_INT_FLAG(TIMER5) == 1)
    {
        /* Clear Timer5 PWM compare up interrupt flag */
        TPWM_CLEAR_CMP_UP_INT_FLAG(TIMER5);

        g_au32TMRINTCount[5]++;
    }

    if(TPWM_GET_PWMINT_WAKEUP_STATUS(TIMER5) == 1)
    {
        TPWM_CLEAR_PWMINT_WAKEUP_STATUS(TIMER5);
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
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR4_MODULE);
    CLK_EnableModuleClock(TMR5_MODULE);
    CLK_SetModuleClock(TMR4_MODULE, CLK_CLKSEL3_TMR4SEL_MIRC, 0);
    CLK_SetModuleClock(TMR5_MODULE, CLK_CLKSEL3_TMR5SEL_LIRC, 0);

    /* Set Timer4 PWM CH0(TM4) and Timer5 PWM CH1(TM5_EXT1) pins */
    SYS->GPB_MFPL &= ~(TM4_PB3_Msk);
    SYS->GPB_MFPL |= TM4_PB3;
    SYS->GPB_MFPH &= ~(TM5_EXT_PB12_Msk);
    SYS->GPB_MFPH |= TM5_EXT_PB12;
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
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------------------------------+\n");
    printf("|    Timer4 and Timer5 PWM Power-down and Wake-up Sample Code    |\n");
    printf("+----------------------------------------------------------------+\n\n");

    printf("# Timer4 PWM_CH0 output frequency is 1000 Hz and duty is 50%%.\n");
    printf("    - Enable period point interrupt\n");
    printf("# Timer5 PWM_CH1 output frequency is 100 Hz and duty is 60%%.\n");
    printf("    - Enable compare up point interrupt\n");
    printf("# I/O configuration:\n");
    printf("    - Timer4 PWM_CH0 on PB.3 \n");
    printf("    - Timer5 PWM_CH1 on PB.12\n");
    printf("# System will enter to power-down while TIMER4 period interrupt event has reached 200 times.\n");
    printf("  And wake-up by TIMER5 compare up interrupt event.\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER4);
    TPWM_ENABLE_PWM_MODE(TIMER5);

    /* Enable output channel */
    TPWM_ENABLE_OUTPUT(TIMER4, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER5, TPWM_CH1);

    /* Set Timer4 PWM output frequency is 1000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER4, 1000, 50);

    /* Set Timer5 PWM output frequency is 100 Hz, duty 60% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER5, 100, 60);

    /* Enable Timer4 and Timer5 NVIC */
    NVIC_EnableIRQ(TMR4_IRQn);
    NVIC_EnableIRQ(TMR5_IRQn);

    /* Clear Timer4 and Timer5 interrupt counts to 0 */
    g_au32TMRINTCount[4] = g_au32TMRINTCount[5] = 0;

    /* Enable Timer4 PWM period point interrupt */
    TPWM_ENABLE_PERIOD_INT(TIMER4);

    /* Enable Timer5 PWM compare up point interrupt */
    TPWM_ENABLE_CMP_UP_INT(TIMER5);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER4);
    TPWM_START_COUNTER(TIMER5);

    /* System will enter to power-down while TIMER4 period interrupt event has reached 200 times */
    while(g_au32TMRINTCount[4] < 200) {}

    /* Enable Timer5 PWM interrupt wake-up function */
    TPWM_ENABLE_PWMINT_WAKEUP(TIMER5);

    printf("*** System in power-down mode and check the Timer4 and Timer5 PWM output waveform ***\n");
    printf("    - Timer5 PWM wake-up event interval is about 10 ms\n\n");
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if(--u32TimeOutCnt == 0) break;

    while(1)
    {
        /* Unlock protected registers */
        SYS_UnlockReg();

        CLK_PowerDown();
    }
}
