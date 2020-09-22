/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to generate the first booting image, NuBL2.
 *           After NuBL2 runs, NuBL2 will authenticate NuBL32 and NuBL33 then jump to execute in NuBL32.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "NuBL2.h"


static volatile FW_INFO_T  g_NuBL3xFwInfo; // Allocate a FWINFO buffer for storing NuBL32/NuBL33 FWINFO data

void SYS_Init(void);
void UART_Init(void);


static int32_t CheckROTPKStatus(void)
{
    /* Configure module clock */
    CLK_EnableModuleClock(KS_MODULE);

    KS_Open();

    if((KS->OTPSTS&0x3) == 0)
    {
        printf("ROTPK absent! NuBL2 execution via direct boot.\n\n");
        
        // TODO: User should set ROTPK in KS OTP0 and OTP1, 
        //          so that the Secure Bootloader(NuBL1) can perform trusted boot to execute NuBL2.
        
        /*
            The NuBL2 ROTPK is a set of ECC P-256 public key pair.
            If the ECC P-256 private key is 380a67fcfc01ca7073da7c2c54296a61327f77262a7d4674c3d8e29a63e3fa20, refer to FwSign.ini.
            The public key pair should be
                Public Key 1 = 755B3819F05A3E9F32D4D599062834AAC5220F75955378414A8F63716A152CE2
                Public Key 2 = 91C413F1915ED7B47473FD797647BA3D83E8224377909AF5B30C530EAAD79FD7
        
            Notes of programming NuBL2 ECC ROTPK:
            * The ROTPK arrary for KS_WriteOTP API
                const uint32_t g_au32ROTPK[16] = 
                {
                    // Public key 1
                    0x6a152ce2, 0x4a8f6371, 0x95537841, 0xc5220f75,
                    0x062834aa, 0x32d4d599, 0xf05a3e9f, 0x755b3819,
                    // Public key 2
                    0xaad79fd7, 0xb30c530e, 0x77909af5, 0x83e82243, 
                    0x7647ba3d, 0x7473fd79, 0x915ed7b4, 0x91c413f1,
                };
        
            * An example to program KS OPT0 and OPT1
                // Configure module clock
                CLK_EnableModuleClock(KS_MODULE);
                KS_Open();
                KS_WriteOTP(0, (KS_META_256 | KS_META_ECC | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[0]);
                KS_WriteOTP(1, (KS_META_256 | KS_META_ECC | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[8]);
        */
    }
    else
    {
        printf("ROTPK present. NuBL2 execution via secure boot.\n\n");
    }

    return 0;
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

    /* Set SysTick source to HCLK/2*/
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
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
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open((UART_T *)DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32NuBL32Base;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %d Hz (Non-secure flash base: 0x%x)\n", SystemCoreClock, SCU->FNSADDR);
    printf("+------------------------------------------+\n");
    printf("|    SecureBootDemo - NuBL2 Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    CheckROTPKStatus();

    /* Authenticate NuBL32 FW */
    memcpy((void *)((uint32_t)&g_NuBL3xFwInfo), (void *)NUBL32_FW_INFO_BASE, sizeof(FW_INFO_T));
    if(VerifyNuBL3x((uint32_t *)((uint32_t)&g_NuBL3xFwInfo), NUBL32_FW_INFO_BASE) == -1)
    {
        printf("\n\nNuBL2 verifies NuBL32 FAIL.\n");
        while(1) {}
    }
    else
    {
        u32NuBL32Base = g_NuBL3xFwInfo.mData.au32FwRegion[0].u32Start;
        printf("\nNuBL2 verifies NuBL32 FW PASS.\n");
    }


    /* Authenticate NuBL33 FW */
    memcpy((void *)((uint32_t)&g_NuBL3xFwInfo), (void *)NUBL33_FW_INFO_BASE, sizeof(FW_INFO_T));
    if(VerifyNuBL3x((uint32_t *)((uint32_t)&g_NuBL3xFwInfo), NUBL33_FW_INFO_BASE) == -1)
    {
        printf("\n\nNuBL2 verifies NuBL33 FAIL.\n");
        while(1) {}
    }
    else
    {
        printf("\nNuBL2 verifies NuBL33 FW PASS.\n");
    }

    /* Jump to execute NuBL32 FW */
    printf("\nJump to execute NuBL32...\n");
    UART_WAIT_TX_EMPTY((UART_T *)DEBUG_PORT);

    /* Disable all interrupt */
    __set_PRIMASK(1);

    FMC_ENABLE_ISP();
    FMC_SetVectorPageAddr(u32NuBL32Base);

    /* Reset to execute NuBL32 FW */
    NVIC_SystemReset();

    while(1) {}
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
