/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of LXT clock frequency monitor function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

static volatile uint32_t s_u32SpareData = 0;

void TAMPER_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);
void GetActiveLXTandTLIRC32Freq(uint32_t *u32LXTFreq, uint32_t *TLIRC32Freq);

void TAMPER_IRQHandler(void)
{
    uint32_t u32FlagStatus;

    /* Tamper interrupt occurred */
    if(TAMPER_GET_INT_FLAG())
    {
        /* Get tamper interrupt status */
        u32FlagStatus = TAMPER_GET_INT_STATUS();

        if(u32FlagStatus & TAMPER_INTSTS_CLKFAILIF_Msk)
        {
            printf("LXT frequency is abnormal!\n\n");

            /* Disable LXT clock frequency monitor fail interrupt */
            TAMPER_DisableInt(TAMPER_INTEN_CLKFIEN_Msk);

            /* Clear tamper interrupt status */
            TAMPER_CLR_INT_STATUS(u32FlagStatus);
        }

        if(u32FlagStatus & TAMPER_INTSTS_CLKSTOPIF_Msk)
        {
            printf("LXT frequency is almost stopped!\n\n");

            /* Disable LXT clock frequency monitor stop interrupt */
            TAMPER_DisableInt(TAMPER_INTEN_CLKSTOPIEN_Msk);

            /* Clear tamper interrupt status */
            TAMPER_CLR_INT_STATUS(u32FlagStatus);
        }

        /* Check spare register data */
        s_u32SpareData = RTC_READ_SPARE_REGISTER(RTC, 0);
        printf(" SPARE_REGISTER[%d] = 0x%x.\n\n", 0, s_u32SpareData);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC, HXT and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable TAMPER module clock */
    CLK_EnableModuleClock(TAMPER_MODULE);

    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

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
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void GetActiveLXTandTLIRC32Freq(uint32_t *u32LXTFreq, uint32_t *TLIRC32Freq)
{
    uint32_t u32GetCNT, u32Period;

    CLK->APBCLK0 |= (CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_TMR1CKEN_Msk);
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_TMR0SEL_Msk | CLK_CLKSEL1_TMR1SEL_Msk)) | (CLK_CLKSEL1_TMR0SEL_HXT | CLK_CLKSEL1_TMR1SEL_LXT);

    TIMER0->CMP = __HXT;
    TIMER0->CTL = 0;

    TIMER1->CMP = 0xFFFFFF;
    TIMER1->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_CNTEN_Msk;

    TIMER0->INTSTS = TIMER0->INTSTS;
    TIMER1->CNT = 0x555;
    while(1)
    {
        if(TIMER1->CNT == 10)
        {
            TIMER0->CTL = TIMER_CTL_CNTEN_Msk;
            break;
        }
    }
    while(TIMER0->INTSTS == 0);
    u32GetCNT = TIMER1->CNT;
    *u32LXTFreq = u32GetCNT - 10;

    /* Feeds TLIRC32 CKO to PD.1 (set PD.1 as GPIO input mode) */
    TIMER0->CTL = 0;
    TIMER0->CMP = 0xFFFFFF;
    TIMER0->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_CNTEN_Msk;
    CLK_SysTickDelay(10000);
    while(PD1 == 0) {}
    while(PD1 == 1) {}
    TIMER0->CMP = 0xFFFFFF;
    while(PD1 == 0) {}
    while(PD1 == 1) {}
    u32Period = TIMER0->CNT;
    *TLIRC32Freq = (1000000 * 12) / u32Period;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32LXTFreq, u32TLIRC32Freq, u32STDCount;
    S_RTC_TIME_DATA_T sInitTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Enable FMC ISP function */
    FMC_Open();

    /* Enable APROM update function */
    FMC_ENABLE_AP_UPDATE();

    /* Enable User Configuration update function */
    FMC_ENABLE_CFG_UPDATE();

    /* Enable Tamper Domain */
    if(FMC_Read(FMC_USER_CONFIG_3) == 0x5AA5FFFF)
    {
        FMC_Erase(FMC_USER_CONFIG_3);
        SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
        while(1);
    }

    /* OSC32K_tamper TEST CLOCK */
    outpw(0x400002F0, 0x1008F00);

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|   TAMPER Frequency Detection Sample Code   |\n");
    printf("+--------------------------------------------+\n\n");
    printf("Please connect PD.0 and PD.1 to get TLIRC32 frequency.\n");
    printf("The spare register data will be cleared to zero if LXT frequency is abnormal.\n");
    printf("\nStop LXT to test.\n\n");

    GetActiveLXTandTLIRC32Freq(&u32LXTFreq, &u32TLIRC32Freq);

    if(u32LXTFreq > u32TLIRC32Freq)
    {
        u32STDCount = ((1000000000 / u32LXTFreq) * 255) / (1000000000 / u32TLIRC32Freq);
        printf("LXT is faster than TLIRC32. [%dHz > %dHz]\n", u32LXTFreq, u32TLIRC32Freq);
    }
    else
    {
        u32STDCount = ((1000000000 / u32TLIRC32Freq) * 255) / (1000000000 / u32LXTFreq);
        printf("TLIRC32 is faster than LXT. [%dHz > %dHz]\n", u32TLIRC32Freq, u32LXTFreq);
    }

    /* Reset tamper coreblock */
    TAMPER_CORE_RESET();
    TAMPER_CORE_RELEASE();

    /* Set the LXT clock frequency monitor fail/stop boundary value.
       The fail/stop boundary value should be less than u32STDCount.
    */
    TAMPER->CDBR = ((u32STDCount - 2) << TAMPER_CDBR_FAILBD_Pos) | ((u32STDCount - 2) << TAMPER_CDBR_STOPBD_Pos);

    /* Enable LXT clock detection */
    TAMPER_ENABLE_LXTDET();

    /* Clear LXT clock frequency monitor fail/stop interrupt flag */
    TAMPER_CLR_INT_STATUS(TAMPER_INTSTS_CLKFAILIF_Msk | TAMPER_INTSTS_CLKSTOPIF_Msk);

    /* Enable LXT clock frequency monitor fail/stop interrupt */
    TAMPER_EnableInt(TAMPER_INTEN_CLKFIEN_Msk | TAMPER_INTEN_CLKSTOPIEN_Msk);
    NVIC_EnableIRQ(TAMPER_IRQn);

    /* Enable to clear RTC spare register */
    TAMPER_ENABLE_RTCSPCLR();

    /* Open RTC and start counting */
    sInitTime.u32Year       = 2020;
    sInitTime.u32Month      = 6;
    sInitTime.u32Day        = 1;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_MONDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;
    if(RTC_Open(&sInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        goto lexit;
    }

    /* Enable spare register */
    RTC_EnableSpareAccess();
    /* Write spare register */
    RTC_WRITE_SPARE_REGISTER(RTC, 0, 0x12345678);

    /* Check spare register data */
    s_u32SpareData = RTC_READ_SPARE_REGISTER(RTC, 0);
    printf("# SPARE_REGISTER[%d] = 0x%x.\n\n", 0, s_u32SpareData);

lexit:

    /* Wait for LXT clock frequency monitor fail/stop interrupt happened */
    while(1);
}
