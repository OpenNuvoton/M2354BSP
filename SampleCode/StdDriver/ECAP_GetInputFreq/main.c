/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use ECAP interface to get input frequency
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t u32Status;
static volatile uint32_t u32IC0Hold;

void TMR0_IRQHandler(void);
void ECAP0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);
void ECAP0_Init(void);
void Timer0_Init(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Timer0 IRQ Handler                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        /*PC.2 gpio toggle */
        PC2 ^= 1;

    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  ECAP0 IRQ Handler                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void ECAP0_IRQHandler(void)
{
    /* Get input Capture status */
    u32Status = ECAP_GET_INT_STATUS(ECAP0);

    /* Check input capture channel 0 flag */
    if((u32Status & ECAP_STATUS_CAPTF0_Msk) == ECAP_STATUS_CAPTF0_Msk)
    {
        /* Clear input capture channel 0 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF0_Msk);

        /* Get input capture counter hold value */
        u32IC0Hold = ECAP0->HLD0;
    }

    /* Check input capture channel 1 flag */
    if((u32Status & ECAP_STATUS_CAPTF1_Msk) == ECAP_STATUS_CAPTF1_Msk)
    {
        /* Clear input capture channel 1 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF1_Msk);
    }

    /* Check input capture channel 2 flag */
    if((u32Status & ECAP_STATUS_CAPTF2_Msk) == ECAP_STATUS_CAPTF2_Msk)
    {
        /* Clear input capture channel 2 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF2_Msk);
    }

    /* Check input capture compare-match flag */
    if((u32Status & ECAP_STATUS_CAPCMPF_Msk) == ECAP_STATUS_CAPCMPF_Msk)
    {
        /* Clear input capture compare-match flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPCMPF_Msk);
    }

    /* Check input capture overflow flag */
    if((u32Status & ECAP_STATUS_CAPTF2_Msk) == ECAP_STATUS_CAPTF2_Msk)
    {
        /* Clear input capture overflow flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPOVF_Msk);
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

    /* Enable ECAP0 module clock */
    CLK_EnableModuleClock(ECAP0_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select TIMER0 module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Set PE.8 for ECAP0_IC0*/
    SYS->GPE_MFPH &= ~SYS_GPE_MFPH_PE8MFP_Msk;
    SYS->GPE_MFPH |= SYS_GPE_MFPH_PE8MFP_ECAP0_IC0;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void ECAP0_Init(void)
{
    /* Enable ECAP0*/
    ECAP_Open(ECAP0, ECAP_DISABLE_COMPARE);

    /* Select Reload function */
    ECAP_SET_CNT_CLEAR_EVENT(ECAP0, (ECAP_CTL1_CAP0RLDEN_Msk | ECAP_CTL1_CAP1RLDEN_Msk));

    /* Enable ECAP0 Input Channel 0*/
    ECAP_ENABLE_INPUT_CHANNEL(ECAP0, ECAP_CTL0_IC0EN_Msk);

    /* Enable ECAP0 source from IC0 */
    ECAP_SEL_INPUT_SRC(ECAP0, ECAP_IC0, ECAP_CAP_INPUT_SRC_FROM_IC);

    /* Select IC0 detect rising edge */
    ECAP_SEL_CAPTURE_EDGE(ECAP0, ECAP_IC0, ECAP_RISING_EDGE);

    /* Input Channel 0 interrupt enabled */
    ECAP_EnableINT(ECAP0, ECAP_CTL0_CAPIEN0_Msk);
}

void Timer0_Init(void)
{

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 10000);
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32Hz = 0, u32Hz_DET = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n");
    printf("+----------------------------------------------------------+\n");
    printf("|        Enhanced Input Capture Timer Driver Sample Code   |\n");
    printf("+----------------------------------------------------------+\n");
    printf("\n");
    printf("  !! GPIO PC.2 toggle periodically    !!\n");
    printf("  !! Connect PC.2 --> PE.8(ECAP0_IC0) !!\n\n");
    printf("     Press any key to start test\n\n");

    getchar();

    /* Initial ECAP0 function */
    ECAP0_Init();

    /* Initial Timer0 function */
    Timer0_Init();

    /* Configure PC.2 as output mode */
    GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    /* Delay 200ms */
    CLK_SysTickDelay(200000);

    /* Init & clear ECAP interrupt status flags */
    u32Status = ECAP_GET_INT_STATUS(ECAP0);
    ECAP0->STATUS = u32Status;

    /* ECAP_CNT starts up-counting */
    ECAP_CNT_START(ECAP0);

    while(1)
    {
        if(u32Status != 0)
        {
            /* Input Capture status is changed, and get a new hold value of input capture counter */
            u32Status = 0;

            /* Calculate the IC0 input frequency */
            u32Hz_DET = (SystemCoreClock / 2) / (u32IC0Hold + 1);


            if(u32Hz != u32Hz_DET)
            {
                /* If IC0 input frequency is changed, Update frquency */
                u32Hz = u32Hz_DET;
            }
            else
            {
                printf("\nECAP0_IC0 input frequency is %d (Hz),u32IC0Hold=0x%08x\n", u32Hz, u32IC0Hold);
                TIMER_Stop(TIMER0); //Disable timer Counting.
                break;
            }
        }

    }
    /* Disable External Interrupt */
    NVIC_DisableIRQ(ECAP0_IRQn);
    NVIC_DisableIRQ(TMR0_IRQn);


    /* Disable ECAP funtion */
    ECAP_Close(ECAP0);

    /* Disable Timer0 IP clock */
    CLK_DisableModuleClock(TMR0_MODULE);

    /* Disable ECAP IP clock */
    CLK_DisableModuleClock(ECAP0_MODULE);

    printf("\nExit ECAP sample code\n");

    while(1);
}



