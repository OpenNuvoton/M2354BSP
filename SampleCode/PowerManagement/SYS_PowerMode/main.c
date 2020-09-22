/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to set different core voltage and main voltage regulator type.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_96MHZ


extern int IsDebugFifoEmpty(void);
static volatile uint8_t g_u8IsINTEvent;

void WDT_IRQHandler(void);
void PowerDownFunction(void);
int32_t pi(void);
void CheckSystemWork(void);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void WDT_IRQHandler(void)
{

    if(WDT_GET_TIMEOUT_INT_FLAG())
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }

    if(WDT_GET_TIMEOUT_WAKEUP_FLAG())
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }

    g_u8IsINTEvent = 1;

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* To check if all the debug messages are finished */
    while(IsDebugFifoEmpty() == 0);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple calculation test function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PI_NUM  256
static int32_t g_ai32f[PI_NUM + 1];
static uint32_t g_au32piTbl[19] =
{
    3141,
    5926,
    5358,
    9793,
    2384,
    6264,
    3383,
    2795,
    288,
    4197,
    1693,
    9937,
    5105,
    8209,
    7494,
    4592,
    3078,
    1640,
    6284
};

static int32_t g_qi32piResult[19];

int32_t pi(void)
{
    int32_t i, i32Err;
    int32_t a = 10000, b = 0, c = PI_NUM, d = 0, e = 0, g = 0;

    for(; b - c;)
        g_ai32f[b++] = a / 5;

    i = 0;
    for(; (void)(d = 0), g = c * 2; c -= 14, g_qi32piResult[i++] = e + d / a, e = d % a)
    {
        if(i == 19)
            break;

        for(b = c; (void)(d += g_ai32f[b] * a), (void)(g_ai32f[b] = d % --g), (void)(d /= g--), --b; d *= b);
    }
    i32Err = 0;
    for(i = 0; i < 19; i++)
    {
        if(g_au32piTbl[i] != (uint32_t)g_qi32piResult[i])
            i32Err = -1;
    }

    return i32Err;
}

void CheckSystemWork(void)
{
    if(pi())
    {
        printf("[FAIL]\n");
    }
    else
    {
        printf("[OK]\n");
    }
}


void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable SRAM module clock */
    CLK_EnableModuleClock(SRAM0_MODULE);
    CLK_EnableModuleClock(SRAM1_MODULE);
    CLK_EnableModuleClock(SRAM2_MODULE);

    /* Enable GPIO module clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Enable UART and WDT module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);

    /* Select UART and WDT module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|         Power Mode Sample Code        |\n");
    printf("+---------------------------------------+\n");

    /* Unlock protected registers before setting power level and main voltage regulator type */
    SYS_UnlockReg();

    /* Set HCLK clock as MIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_MIRC, CLK_CLKDIV0_HCLK(1));
    
    /* Set power level to 0.9V */
    printf("Set power level to 0.9V ");
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL3);

    /* Check system work */
    CheckSystemWork();

    /* Set power level to 1.26V */
    printf("Set power level to 1.26V ");
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL0);

    /* Set core clock as 96MHz from PLL */
    CLK_SetCoreClock(96000000);

    /* Check system work */
    CheckSystemWork();

    /* Set core clock as 84MHz from PLL */
    CLK_SetCoreClock(84000000);

    /* Set power level to 1.2V */
    printf("Set power level to 1.2V ");
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL1);

    /* Check system work */
    CheckSystemWork();

    /* Set core clock as 48MHz from PLL */
    CLK_SetCoreClock(48000000);

    /* Set power level to 1.1V */
    printf("Set power level to 1.1V ");
    SYS_SetPowerLevel(SYS_PLCTL_PLSEL_PL2);

    /* Check system work */
    CheckSystemWork();

    /* Set main voltage regulator type to DCDC mode */
    printf("Set main voltage regulator type to DCDC mode ");

    if(SYS_SetPowerRegulator(SYS_PLCTL_MVRS_DCDC) == 0)
        printf("[no inductor connect]\n");
    else
        CheckSystemWork();      /* Check system work */

    /* Set main voltage regulator type to LDO mode */
    printf("Set main voltage regulator type to LDO mode ");
    SYS_SetPowerRegulator(SYS_PLCTL_MVRS_LDO);

    /* Check system work */
    CheckSystemWork();

    /* Enter to Power-down Mode and wake-up by WDT in LDO mode */
    printf("Enter to Power-down Mode and wake-up ");

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT_TIMEOUT_2POW14, (uint32_t)NULL, FALSE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Check if WDT time-out interrupt and wake-up occurred or not */
    while(g_u8IsINTEvent == 0);

    /* Check system work */
    CheckSystemWork();

    printf("Sample code end.\n");

    while(1);

}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
