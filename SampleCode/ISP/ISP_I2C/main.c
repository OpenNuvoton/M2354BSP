/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 * @date     24, August, 2020
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include <string.h>
#include "targetdev.h"
#include "i2c_transfer.h"

uint32_t g_u32Pclk0;
uint32_t g_u32Pclk1;

void ProcessHardFault(void);
void SH_Return(void);
void SendChar_ToUART(void);
void SYS_Init(void);


void ProcessHardFault(void) {}
void SH_Return(void) {}
void SendChar_ToUART(void) {}


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC first */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 96MHz from HIRC */
    CLK->PLLCTL = CLK_PLLCTL_96MHz_HIRC;

    /* Wait for PLL clock ready */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    /* Set power level by HCLK working frequency */
    SYS->PLCTL = (SYS->PLCTL & (~SYS_PLCTL_PLSEL_Msk)) | SYS_PLCTL_PLSEL_PL0;

    /* Set flash access cycle by HCLK working frequency */
    FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (4);

    /* Select HCLK clock source as PLL and HCLK source divider as 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    PllClock        = 96000000;
    SystemCoreClock = 96000000;
    CyclesPerUs     = SystemCoreClock / 1000000;  /* For CLK_SysTickDelay() */

    /* Enable UART0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART0 module clock source as HIRC */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_UART0SEL_Msk)) | CLK_CLKSEL2_UART0SEL_HIRC;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Enable I2C1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_I2C1CKEN_Msk;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for I2C1 SDA and SCL */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_I2C1_SDA | SYS_GPA_MFPL_PA3MFP_I2C1_SCL);

    /* I2C clock pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN3_Msk;
}

int main(void)
{
    uint32_t au8CmdBuff[16];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Enable ISP */
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);

    /* Get Secure and Non-secure Flash size */
    g_u32ApromSize = GetApromSize();
    g_u32DataFlashAddr = SCU->FNSADDR;

    if(g_u32DataFlashAddr < g_u32ApromSize)
    {
        g_u32DataFlashSize = (g_u32ApromSize - g_u32DataFlashAddr);
    }
    else
    {
        g_u32DataFlashSize = 0;
    }

    /* Init I2C */
    I2C_Init();

    /* Set Systick time-out as 300ms */
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Wait for CMD_CONNECT command until Systick time-out */
    while(1)
    {
        /* Wait for CMD_CONNECT command */
        if(g_u8I2cDataReady == 1)
        {
            goto _ISP;
        }
        /* Systick time-out, go to APROM */
        if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    /* Parse command from master and send response back */
    while(1)
    {
        if(g_u8I2cDataReady == 1)
        {
            /* Get command from I2C receive buffer */
            memcpy(au8CmdBuff, g_au8I2cRcvBuf, 64);
            g_u8I2cDataReady = 0;
            /* Parse the current command */
            ParseCmd((unsigned char *)au8CmdBuff, 64);
        }
    }

_APROM:

    /* Reset system and boot from APROM */
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while(1);
}
