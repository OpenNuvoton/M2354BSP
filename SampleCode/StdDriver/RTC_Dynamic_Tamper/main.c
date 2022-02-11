/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate the RTC dynamic tamper function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32IsTamper = FALSE;

void RTC_TAMPER_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);


/**
 * @brief       IRQ Handler for RTC TAMPER Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTC_TAMPER_IRQHandler is default IRQ of RTC TAMPER, declared in startup_M2354.s.
 */
void RTC_TAMPER_IRQHandler(void)
{
    uint32_t u32FlagStatus, u32TAMPCAL, u32TAMPTIME;
    uint32_t i;

    /* Tamper interrupt occurred */
    if(RTC_GET_TAMPER_INT_FLAG(RTC))
    {
        u32FlagStatus = RTC_GET_TAMPER_INT_STATUS(RTC);

        for(i = 0; i < 6; i++)
        {
            if(u32FlagStatus & (0x1UL << (i + RTC_INTSTS_TAMP0IF_Pos)))
                printf(" Tamper %d Detected!!\n", i);
        }

        u32TAMPCAL = RTC->TAMPCAL;
        u32TAMPTIME = RTC->TAMPTIME;
        printf(" Tamper detected date/time: 20%d%d/%d%d/%d%d ",
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_TENYEAR_Msk) >> RTC_TAMPCAL_TENYEAR_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_YEAR_Msk) >> RTC_TAMPCAL_YEAR_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_TENMON_Msk) >> RTC_TAMPCAL_TENMON_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_MON_Msk) >> RTC_TAMPCAL_MON_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_TENDAY_Msk) >> RTC_TAMPCAL_TENDAY_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_DAY_Msk) >> RTC_TAMPCAL_DAY_Pos));
        printf("%d%d:%d%d:%d%d.\n\n",
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_TENHR_Msk) >> RTC_TAMPTIME_TENHR_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_HR_Msk) >> RTC_TAMPTIME_HR_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_TENMIN_Msk) >> RTC_TAMPTIME_TENMIN_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_MIN_Msk) >> RTC_TAMPTIME_MIN_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_TENSEC_Msk) >> RTC_TAMPTIME_TENSEC_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_SEC_Msk) >> RTC_TAMPTIME_SEC_Pos));

        RTC_CLEAR_TAMPER_INT_FLAG(RTC, u32FlagStatus);
        g_u32IsTamper = TRUE;
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
    /* Enable LXT-32KHz */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable RTC module clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Set multi-function pins for RTC Tamper */
    SYS->GPF_MFPL &= ~(TAMPER0_PF6_Msk | TAMPER1_PF7_Msk);
    SYS->GPF_MFPH &= ~(TAMPER2_PF8_Msk | TAMPER3_PF9_Msk | TAMPER4_PF10_Msk | TAMPER5_PF11_Msk);
    SYS->GPF_MFPL |= (TAMPER0_PF6 | TAMPER1_PF7);
    SYS->GPF_MFPH |= (TAMPER2_PF8 | TAMPER3_PF9 | TAMPER4_PF10 | TAMPER5_PF11);
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
    S_RTC_TIME_DATA_T sInitTime, sGetTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------+\n");
    printf("|    RTC Dynamic Tamper Sample Code   |\n");
    printf("+-------------------------------------+\n\n");

    /* Set LXT as RTC clock source */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Open RTC and start counting */
    sInitTime.u32Year       = 2019;
    sInitTime.u32Month      = 12;
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
        while(1);
    }

    RTC_GetDateAndTime(&sGetTime);
    printf("# Initial data/time is: %d/%02d/%02d %02d:%02d:%02d.\n",
           sGetTime.u32Year, sGetTime.u32Month, sGetTime.u32Day, sGetTime.u32Hour, sGetTime.u32Minute, sGetTime.u32Second);
    printf("# Please connect (tamper0 & tamper1) and (tamper2 & tamper3) and (tamper4 & tamper5) first.\n");
    printf("                    (PF.6 to PF.7)          (PF.8 to PF.9)         (PF.10 to PF.11)\n");
    printf("# Press any key to start test:\n\n");
    getchar();

    printf("# Check tamper date/time when tamper event occurred:\n\n");

    RTC_CLEAR_TAMPER_INT_FLAG(RTC, RTC_INTSTS_TAMP0IF_Msk | RTC_INTSTS_TAMP1IF_Msk | RTC_INTSTS_TAMP2IF_Msk |
                              RTC_INTSTS_TAMP3IF_Msk | RTC_INTSTS_TAMP4IF_Msk | RTC_INTSTS_TAMP5IF_Msk);

    RTC_DynamicTamperEnable(RTC_PAIR0_SELECT | RTC_PAIR1_SELECT | RTC_PAIR2_SELECT, RTC_TAMPER_DEBOUNCE_ENABLE, 0, 0);
    RTC_DynamicTamperConfig(RTC_2POW10_CLK, 1, 0, RTC_REF_RANDOM_PATTERN);

    g_u32IsTamper = FALSE;

    /* Enable RTC Tamper Interrupt */
    RTC_EnableInt(RTC_INTEN_TAMP1IEN_Msk | RTC_INTEN_TAMP3IEN_Msk | RTC_INTEN_TAMP5IEN_Msk);
    NVIC_EnableIRQ(RTC_TAMPER_IRQn);

    while(1)
    {
        while(g_u32IsTamper == FALSE) {}
        g_u32IsTamper = FALSE;
    }
}
