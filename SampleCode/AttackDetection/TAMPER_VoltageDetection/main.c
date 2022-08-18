/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of TAMPER voltage detection function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void);
void UART_Init(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LXTSTB_Msk);

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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
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

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|   TAMPER Voltage Detection Sample Code   |\n");
    printf("+------------------------------------------+\n\n");
    printf("System will reset if any voltage detection interrupt is detected.\n\n");
    printf("Press any key to start test.\n");
    getchar();
    printf("\n");

    /* Reset tamper coreblock */
    TAMPER_CORE_RESET();
    TAMPER_CORE_RELEASE();

    /* Enable over voltage detector and wait until stable */
    SYS_UnlockReg();
    SYS->OVDCTL = SYS_OVDCTL_OVDEN_Msk;
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(SYS->OVDCTL & SYS_OVDCTL_OVDSTB_Msk))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for over voltage detector stable time-out!\n");
            goto lexit;
        }
    }
    SYS_LockReg();

    /* Enable battery loss detector */
    RTC->TEST = RTC_TEST_BATDETEN_Msk;
    SYS_UnlockReg();
    SYS->BATLDCTL = SYS_BATLDCTL_BATLDEN_Msk;
    SYS_LockReg();

    /* Initialize the trim value of under-shoot and over-shoot detection level */
    TAMPER_TLVD_TRIM_INIT(TAMPER_LBSTRIM_TLVDSEL_0_90V);
    TAMPER_TOVD_TRIM_INIT(TAMPER_LBSTRIM_TOVDSEL_1_40V);

    /* Clear different voltage interrupt flag */
    TAMPER_CLR_INT_STATUS(TAMPER_INTSTS_OVPOUTIF_Msk | TAMPER_INTSTS_VBATLOSSIF_Msk | TAMPER_INTSTS_BODIF_Msk);

    /* Enable different voltage detection interrupt */
    TAMPER_EnableInt(TAMPER_INTEN_OVPIEN_Msk | TAMPER_INTEN_RTCLVRIEN_Msk | TAMPER_INTEN_VLOSSIEN_Msk | TAMPER_INTEN_BODIEN_Msk);

    /* Enable to trigger chip reset */
    TAMPER_ENABLE_CHIPRST();

lexit:

    /* Wait for voltage detection interrupt happened */
    while(1);
}
