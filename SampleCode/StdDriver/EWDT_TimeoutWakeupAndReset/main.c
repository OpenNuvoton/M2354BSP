/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement EWDT time-out interrupt event to wake up system and generate time-out reset system event while EWDT time-out reset delay period expired.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8IsINTEvent;
static volatile uint32_t g_u32WakeupCounts;

void EWDT_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);


/**
 * @brief       IRQ Handler for EWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The EWDT_IRQHandler is default IRQ of EWDT, declared in startup_M2354.s.
 */
void EWDT_IRQHandler(void)
{    
    if(g_u32WakeupCounts < 10)
    {
        EWDT_RESET_COUNTER();
    }

    if(EWDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear EWDT time-out interrupt flag */
        EWDT_CLEAR_TIMEOUT_INT_FLAG();

        g_u8IsINTEvent = 1;
    }

    if(EWDT_GET_TIMEOUT_WAKEUP_FLAG() == 1)
    {
        /* Clear EWDT time-out wake-up flag */
        EWDT_CLEAR_TIMEOUT_WAKEUP_FLAG();

        g_u32WakeupCounts++;
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

    /* Enable LIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

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

    /* Enable EWDT module clock */
    CLK_EnableModuleClock(EWDT_MODULE);
    CLK_SetModuleClock(EWDT_MODULE, CLK_CLKSEL1_EWDTSEL_LIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;
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
    printf("+------------------------------------------+\n");
    printf("|    EWDT Time-out Wake-up Sample Code     |\n");
    printf("+------------------------------------------+\n\n");

    /* To check if system has been reset by EWDT time-out reset or not */
    if(EWDT_GET_RESET_FLAG() == 1)
    {
        EWDT_CLEAR_RESET_FLAG();
        printf("*** System has been reset by EWDT time-out event ***\n\n");
        while(1) {}
    }

    printf("# EWDT Settings:\n");
    printf("    - Clock source is LIRC                  \n");
    printf("    - Time-out interval is 2^14 * EWDT clock\n");
    printf("      (around 0.5 s)                        \n");
    printf("    - Interrupt enable                      \n");
    printf("    - Wake-up function enable               \n");
    printf("    - Reset function enable                 \n");
    printf("    - Reset delay period is 18 * EWDT clock \n");
    printf("# System will generate a EWDT time-out interrupt event after 0.5 s, \n");
    printf("    and will be wake up from power-down mode also.\n");
    printf("    (Use PA.2 high/low period time to check EWDT time-out interval)\n");
    printf("# When EWDT interrupt counts large than 10, \n");
    printf("    we will not reset EWDT counter value and system will be reset immediately by EWDT time-out reset signal.\n");

    /* Use PA.2 to check time-out interrupt period time */
    PA->MODE = (PA->MODE & ~GPIO_MODE_MODE2_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE2_Pos);
    PA2 = 1;

    /* Enable EWDT NVIC */
    NVIC_EnableIRQ(EWDT_IRQn);

    /* Because of all bits can be written in EWDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Configure EWDT settings and start EWDT counting */
    g_u8IsINTEvent = g_u32WakeupCounts = 0;
    EWDT_Open(EWDT_TIMEOUT_2POW14, EWDT_RESET_DELAY_18CLK, TRUE, TRUE);

    /* Enable EWDT interrupt function */
    EWDT_EnableInt();

    while(1)
    {
        /* System enter to Power-down */
        /* To program PWRCTL register, it needs to disable register protection first. */
        SYS_UnlockReg();
        printf("\nSystem enter to power-down mode ...\n");
        /* To check if all the debug messages are finished */
        while(!UART_IS_TX_EMPTY(DEBUG_PORT));
        
        CLK_PowerDown();

        /* Check if EWDT time-out interrupt and wake-up occurred or not */
        while(g_u8IsINTEvent == 0) {}
        PA2 ^= 1;

        g_u8IsINTEvent = 0;
        printf("System has been waken up done. EWDT wake-up counts: %d.\n\n", g_u32WakeupCounts);
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
