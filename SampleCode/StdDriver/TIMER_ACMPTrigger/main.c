/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use ACMP to trigger Timer0 counter reset mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void TMR0_IRQHandler(void);
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
    if(TIMER_GetCaptureIntFlag(TIMER0))
    {
        printf("ACMP triggered timer reset while counter is at %d.\n", TIMER_GetCaptureData(TIMER0));

        /* Clear timer capture interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER0);
    }
}

void SYS_Init(void)
{
    /* Enable all GPIO and SRAM clock */
    CLK->AHBCLK |= (CLK_AHBCLK_TRACECKEN_Msk | CLK_AHBCLK_SRAM0CKEN_Msk | CLK_AHBCLK_SRAM1CKEN_Msk | CLK_AHBCLK_SRAM2CKEN_Msk |
                    CLK_AHBCLK_GPACKEN_Msk | CLK_AHBCLK_GPBCKEN_Msk | CLK_AHBCLK_GPCCKEN_Msk | CLK_AHBCLK_GPDCKEN_Msk |
                    CLK_AHBCLK_GPECKEN_Msk | CLK_AHBCLK_GPFCKEN_Msk | CLK_AHBCLK_GPGCKEN_Msk | CLK_AHBCLK_GPHCKEN_Msk);
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(TRACE_CLK_PE12_Msk | TRACE_DATA0_PE11_Msk | TRACE_DATA1_PE10_Msk | TRACE_DATA2_PE9_Msk | TRACE_DATA3_PE8_Msk)) |
                    (TRACE_CLK_PE12 | TRACE_DATA0_PE11 | TRACE_DATA1_PE10 | TRACE_DATA2_PE9 | TRACE_DATA3_PE8);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_96MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
    
    /* Select HCLK clock source as PLL and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set SysTick source to HCLK/2 */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Enable ACMP module clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set PB.4 multi-function pin for ACMP1 positive input pin */
    SYS->GPB_MFPL &= ~ACMP1_P1_PB4_Msk;
    SYS->GPB_MFPL |= ACMP1_P1_PB4;

    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));
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
    printf("+----------------------------------------------------+\n");
    printf("|    ACMP Trigger Timer Counter Reset Sample Code    |\n");
    printf("+----------------------------------------------------+\n\n");

    /* Set PB.5 to output mode and set state to high first */
    PB5 = 1;
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE5_Msk) | (0x1 << GPIO_MODE_MODE5_Pos);

    printf("This sample code demonstrate ACMP trigger timer counter reset mode.\n");
    printf("Please connect PB.5 with ACMP1 positive input pin PB.4, press any key to continue.\n\n");
    getchar();

    /* Give a dummy target frequency here. Will over write capture resolution with macro */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000);

    /* Update prescale to set proper resolution */
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);

    /* Set compare value as large as possible, so don't need to worry about counter overrun too frequently */
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);

    /* Configure Timer 0 free counting mode */
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_COUNTER_RESET_MODE, TIMER_CAPTURE_EVENT_RISING);
    /* Set capture source from ACMP */
    TIMER0->CTL |= TIMER_CTL_CAPSRC_Msk;
    /* Set capture source from ACMP1 */
    TIMER0->EXTCTL |= TIMER_INTER_CAPTURE_SOURCE_ACMP1;

    /* Start Timer 0 */
    TIMER_Start(TIMER0);

    /* Configure ACMP1. Enable ACMP1 and select band-gap voltage as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Select P1 as ACMP1 positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);
    /* Enable timer interrupt */
    TIMER_EnableCaptureInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    while(1)
    {
        PB5 = 0; // low
        CLK_SysTickDelay(10000);
        PB5 = 1;  // high
        CLK_SysTickDelay(10000);
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
