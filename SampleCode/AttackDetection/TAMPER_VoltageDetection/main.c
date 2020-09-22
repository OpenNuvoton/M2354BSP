/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of TAMPER voltage detection function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
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

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_96MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select power level as level 0 */
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL0);

    /* Select HCLK clock source as PLL and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable TAMPER module clock */
    CLK_EnableModuleClock(TAMPER_MODULE);

    /* Enable CRPT module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

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

    TAMPER_CLR_INT_STATUS(TAMPER_INTSTS_OVPOUTIF_Msk | TAMPER_INTSTS_BODIF_Msk);

    /* Enable over voltage detector and wait until stable */
    SYS_UnlockReg();
    SYS->OVDCTL = SYS_OVDCTL_OVDEN_Msk;
    while(!(SYS->OVDCTL & SYS_OVDCTL_OVDSTB_Msk));
    SYS_LockReg();

    /* Enable different voltage detection interrupt */
    TAMPER_EnableInt(TAMPER_INTEN_OVPIEN_Msk | TAMPER_INTEN_RTCLVRIEN_Msk | TAMPER_INTEN_BODIEN_Msk);

    /* Enable to trigger chip reset */
    TAMPER_ENABLE_CHIPRST();

    /* Wait for voltage detection interrupt happened */
    while(1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
