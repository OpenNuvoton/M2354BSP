/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use the capture function to capture the internal clock frequency.
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
    /* Enable HXT and LIRC */
    CLK_EnableXtalRC((CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LIRCEN_Msk));

    /* Waiting for clock ready */
    CLK_WaitClockReady((CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LIRCSTB_Msk));

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR4_MODULE);
    CLK_EnableModuleClock(TMR5_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR4_MODULE, CLK_CLKSEL3_TMR4SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR5_MODULE, CLK_CLKSEL3_TMR5SEL_PCLK0, 0);
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
    uint32_t au32CAPValue[6];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------------------+\n");
    printf("|    Capture Internal Clcok Frequency Sample Code    |\n");
    printf("+----------------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HCLK, and operation at periodic mode\n");
    printf("    - Capture source is HXT and source divider 128\n");
    printf("# Timer1 Settings:\n");
    printf("    - Clock source is HCLK, and operation at periodic mode\n");
    printf("    - Capture source is HIRC and source divider 256\n");
    printf("# Timer4 Settings:\n");
    printf("    - Clock source is HCLK, and operation at periodic mode\n");
    printf("    - Capture source is LIRC and source divider 1\n");
    printf("# Timer5 Settings:\n");
    printf("    - Clock source is HCLK, and operation at periodic mode\n");
    printf("    - Capture source is MIRC and source divider 32\n");
    printf("# Calculate the target clock frequency:\n\n");

    /* Open Timer0, Timer1, Timer4 and Timer5 operation in periodic mode */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1);
    TIMER_Open(TIMER4, TIMER_PERIODIC_MODE, 1);
    TIMER_Open(TIMER5, TIMER_PERIODIC_MODE, 1);

    /* Set no presacle and compare value to maximum 0xFFFFFFul */
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFFUL);
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);
    TIMER_SET_CMP_VALUE(TIMER1, 0xFFFFFFUL);
    TIMER_SET_PRESCALE_VALUE(TIMER4, 0);
    TIMER_SET_CMP_VALUE(TIMER4, 0xFFFFFFUL);
    TIMER_SET_PRESCALE_VALUE(TIMER5, 0);
    TIMER_SET_CMP_VALUE(TIMER5, 0xFFFFFFUL);

    /* Select capture source from internal signal */
    TIMER0->CTL = ((TIMER0->CTL & ~TIMER_CTL_CAPSRC_Msk) | TIMER_CAPTURE_SOURCE_FROM_INTERNAL);
    TIMER1->CTL = ((TIMER1->CTL & ~TIMER_CTL_CAPSRC_Msk) | TIMER_CAPTURE_SOURCE_FROM_INTERNAL);
    TIMER4->CTL = ((TIMER4->CTL & ~TIMER_CTL_CAPSRC_Msk) | TIMER_CAPTURE_SOURCE_FROM_INTERNAL);
    TIMER5->CTL = ((TIMER5->CTL & ~TIMER_CTL_CAPSRC_Msk) | TIMER_CAPTURE_SOURCE_FROM_INTERNAL);

    /* Enable capture function */
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_COUNTER_RESET_MODE, TIMER_CAPTURE_EVENT_FALLING);
    TIMER_EnableCapture(TIMER1, TIMER_CAPTURE_COUNTER_RESET_MODE, TIMER_CAPTURE_EVENT_FALLING);
    TIMER_EnableCapture(TIMER4, TIMER_CAPTURE_COUNTER_RESET_MODE, TIMER_CAPTURE_EVENT_FALLING);
    TIMER_EnableCapture(TIMER5, TIMER_CAPTURE_COUNTER_RESET_MODE, TIMER_CAPTURE_EVENT_FALLING);

    /* Select capture source and source divider */
    TIMER0->EXTCTL |= (TIMER_CAPTURE_SOURCE_DIV_256 | TIMER_INTER_CAPTURE_SOURCE_HXT);
    TIMER1->EXTCTL |= (TIMER_CAPTURE_SOURCE_DIV_128 | TIMER_INTER_CAPTURE_SOURCE_HIRC);
    TIMER4->EXTCTL |= (TIMER_CAPTURE_SOURCE_DIV_1 | TIMER_INTER_CAPTURE_SOURCE_LIRC);
    TIMER5->EXTCTL |= (TIMER_CAPTURE_SOURCE_DIV_32 | TIMER_INTER_CAPTURE_SOURCE_MIRC);

    /* Start timer capture function */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER1);
    TIMER_Start(TIMER4);
    TIMER_Start(TIMER5);

    /* Delay 10 ms to get captured data */
    CLK_SysTickDelay(10000);

    /* Get TIMER0 captured data */
    TIMER_ClearCaptureIntFlag(TIMER0);
    while(TIMER_GetCaptureIntFlag(TIMER0) == 0) {}
    TIMER_ClearCaptureIntFlag(TIMER0);
    au32CAPValue[0] = TIMER_GetCaptureData(TIMER0);

    /* Get TIMER1 captured data */
    TIMER_ClearCaptureIntFlag(TIMER1);
    while(TIMER_GetCaptureIntFlag(TIMER1) == 0) {}
    TIMER_ClearCaptureIntFlag(TIMER1);
    au32CAPValue[1] = TIMER_GetCaptureData(TIMER1);

    /* Get TIMER4 captured data */
    TIMER_ClearCaptureIntFlag(TIMER4);
    while(TIMER_GetCaptureIntFlag(TIMER4) == 0) {}
    TIMER_ClearCaptureIntFlag(TIMER4);
    au32CAPValue[4] = TIMER_GetCaptureData(TIMER4);

    /* Get TIMER5 captured data */
    TIMER_ClearCaptureIntFlag(TIMER5);
    while(TIMER_GetCaptureIntFlag(TIMER5) == 0) {}
    TIMER_ClearCaptureIntFlag(TIMER5);
    au32CAPValue[5] = TIMER_GetCaptureData(TIMER5);

    /*
        Target input clock source is :
            (1000000 / ((CAP_DAT + 1) * (1000000 / TIMER_CLK))) * CAPSRC_DIV
            = (TIMER_CLK / (CAP_DAT + 1))* CAPSRC_DIV
            = (TIMER_CLK * CAPSRC_DIV) / (CAP_DAT + 1) (Hz)
    */
    printf("HXT freq:  %d Hz.\n", ((SystemCoreClock / (au32CAPValue[0] + 1)) * 256));
    printf("HIRC freq: %d Hz.\n", ((SystemCoreClock / (au32CAPValue[1] + 1)) * 128));
    printf("LIRC freq: %d Hz.\n", ((SystemCoreClock / (au32CAPValue[4] + 1)) * 1));
    printf("MIRC freq: %d Hz.\n", ((SystemCoreClock / (au32CAPValue[5] + 1)) * 32));

    /* Stop Timer0, Timer1, Timer4 and Timer5 counting */
    TIMER_Stop(TIMER0);
    TIMER_Stop(TIMER1);
    TIMER_Stop(TIMER4);
    TIMER_Stop(TIMER5);

    while(1) {}
}
