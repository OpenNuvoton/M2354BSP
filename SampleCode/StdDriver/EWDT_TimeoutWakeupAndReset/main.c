/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement EWDT time-out interrupt event to wake up system and generate time-out reset system event while EWDT time-out reset delay period expired.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
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
    /* Enable LIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Enable EWDT module clock */
    CLK_EnableModuleClock(EWDT_MODULE);
    CLK_SetModuleClock(EWDT_MODULE, CLK_CLKSEL1_EWDTSEL_LIRC, 0);
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
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(!UART_IS_TX_EMPTY(DEBUG_PORT))
            if(--u32TimeOutCnt == 0) break;

        CLK_PowerDown();

        /* Check if EWDT time-out interrupt and wake-up occurred or not */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(g_u8IsINTEvent == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for EWDT interrupt time-out!\n");
                break;
            }
        }

        PA2 ^= 1;

        g_u8IsINTEvent = 0;
        printf("System has been waken up done. EWDT wake-up counts: %d.\n\n", g_u32WakeupCounts);
    }
}
