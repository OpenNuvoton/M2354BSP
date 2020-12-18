/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate output different duty waveform in Timer0~Timer5 PWM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART_Init(void);


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
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_EnableModuleClock(TMR4_MODULE);
    CLK_EnableModuleClock(TMR5_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);
    CLK_SetModuleClock(TMR4_MODULE, CLK_CLKSEL3_TMR4SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR5_MODULE, CLK_CLKSEL3_TMR5SEL_PCLK0, 0);

    /* Set Timer0 PWM CH0(TM0), Timer1 PWM CH0(TM1), Timer2 PWM CH0(TM2) and Timer3 PWM CH0(TM3) pins */
    SYS->GPG_MFPL &= ~(TM0_PG2_Msk | TM1_PG3_Msk | TM2_PG4_Msk);
    SYS->GPG_MFPL |= TM0_PG2 | TM1_PG3 | TM2_PG4;
    SYS->GPF_MFPH &= ~TM3_PF11_Msk;
    SYS->GPF_MFPH |= TM3_PF11;
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
    printf("|    Timer0~Timer5 PWM Output Duty Sample Code    |\n");
    printf("+-------------------------------------------------+\n\n");

    printf("# Timer0 PWM_CH0 output frequency is 18000 Hz and duty is 50%%.\n");
    printf("# Timer1 PWM_CH0 output frequency is 10000 Hz and duty is 10%%.\n");
    printf("# Timer2 PWM_CH0 output frequency is  9000 Hz and duty is 75%%.\n");
    printf("# Timer3 PWM_CH0 output frequency is  4000 Hz and duty is 20%%.\n");
    printf("# Timer4 PWM_CH0 output frequency is 10000 Hz and duty is 40%%.\n");
    printf("# Timer5 PWM_CH1 output frequency is  5000 Hz and duty is 60%%.\n");
    printf("# I/O configuration:\n");
    printf("    - Timer0 PWM_CH0 on PG.2 \n");
    printf("    - Timer1 PWM_CH0 on PG.3 \n");
    printf("    - Timer2 PWM_CH0 on PG.4 \n");
    printf("    - Timer3 PWM_CH0 on PF.11\n");
    printf("    - Timer4 PWM_CH0 on PB.3 \n");
    printf("    - Timer5 PWM_CH1 on PB.12\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);
    TPWM_ENABLE_PWM_MODE(TIMER1);
    TPWM_ENABLE_PWM_MODE(TIMER2);
    TPWM_ENABLE_PWM_MODE(TIMER3);
    TPWM_ENABLE_PWM_MODE(TIMER4);
    TPWM_ENABLE_PWM_MODE(TIMER5);

    /* Set PWM mode as independent mode */
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER0);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER1);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER2);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER3);
    /* Timer4 and Timer5 only support independent mode */

    /* Enable output channel */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER1, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER2, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER3, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER4, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER5, TPWM_CH1);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50);

    /* Set Timer1 PWM output frequency is 10000 Hz, duty 10% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER1, 10000, 10);

    /* Set Timer2 PWM output frequency is 9000 Hz, duty 75% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER2, 9000, 75);

    /* Set Timer3 PWM output frequency is 4000 Hz, duty 20% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER3, 4000, 20);

    /* Set Timer4 PWM output frequency is 10000 Hz, duty 40% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER4, 10000, 40);

    /* Set Timer5 PWM output frequency is 5000 Hz, duty 60% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER5, 5000, 60);

    /* Set PWM up count type */
    TPWM_SET_COUNTER_TYPE(TIMER0, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER1, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER2, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER3, TPWM_UP_COUNT);
    /* Timer4 and Timer5 only support up count type */

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER0);
    TPWM_START_COUNTER(TIMER1);
    TPWM_START_COUNTER(TIMER2);
    TPWM_START_COUNTER(TIMER3);
    TPWM_START_COUNTER(TIMER4);
    TPWM_START_COUNTER(TIMER5);

    printf("*** Check Timer0~Timer5 PWM output waveform by oscilloscope ***\n\n");

    while(1) {}
}
