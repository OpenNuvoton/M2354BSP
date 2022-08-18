/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of TAMPER voltage glitch positive and negative detection function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

static uint32_t au32MyAESKey[8] =
{
    0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f,
    0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f
};

static uint32_t au32MyAESIV[4] =
{
    0x00000000, 0x00000000, 0x00000000, 0x00000000
};

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8InputData[] =
{
#else
__attribute__((aligned(4))) static uint8_t au8InputData[] =
{
#endif
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
};

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8OutputData[1024];
#else
__attribute__((aligned(4))) static uint8_t au8OutputData[1024];
#endif

static volatile int32_t  g_AES_done;

void TAMPER_IRQHandler(void);
void CRPT_IRQHandler(void);
void DumpBuffHex(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void UART_Init(void);

void TAMPER_IRQHandler(void)
{
    uint32_t u32FlagStatus;

    /* Tamper interrupt occurred */
    if(TAMPER_GET_INT_FLAG())
    {
        /* Get tamper interrupt status */
        u32FlagStatus = TAMPER_GET_INT_STATUS();

        if(u32FlagStatus & TAMPER_INTSTS_VGPEVIF_Msk)
        {
            /* Clear tamper interrupt status */
            TAMPER_CLR_INT_STATUS(u32FlagStatus);

            printf("LDO_CAP positive glitch is detected!\n\n");
        }

        if(u32FlagStatus & TAMPER_INTSTS_VGNEVIF_Msk)
        {
            /* Clear tamper interrupt status */
            TAMPER_CLR_INT_STATUS(u32FlagStatus);

            printf("LDO_CAP negative glitch is detected!\n\n");
        }

        DumpBuffHex(au8OutputData, sizeof(au8InputData));
    }
}

void CRPT_IRQHandler(void)
{
    if(AES_GET_INT_FLAG(CRPT))
    {
        g_AES_done = 1;
        AES_CLR_INT_FLAG(CRPT);
    }
}

void DumpBuffHex(uint8_t *pucBuff, int nBytes)
{
    int32_t i32Idx, i;

    i32Idx = 0;
    while(nBytes > 0)
    {
        printf("0x%04X  ", i32Idx);
        for(i = 0; i < 16; i++)
            printf("%02x ", pucBuff[i32Idx + i]);
        printf("  ");
        for(i = 0; i < 16; i++)
        {
            if((pucBuff[i32Idx + i] >= 0x20) && (pucBuff[i32Idx + i] < 127))
                printf("%c", pucBuff[i32Idx + i]);
            else
                printf(".");
            nBytes--;
        }
        i32Idx += 16;
        printf("\n");
    }
    printf("\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HIRC48 clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HIRC48EN_Msk);

    /* Wait for HIRC and HIRC48 clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HIRC48STB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable TAMPER module clock */
    CLK_EnableModuleClock(TAMPER_MODULE);

    /* Enable CRPT module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

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
    printf("+--------------------------------------------------+\n");
    printf("|   TAMPER Fault Injection Detection Sample Code   |\n");
    printf("+--------------------------------------------------+\n\n");
    printf("The encryption result will be zero if any voltage fault injection is detected before AES encryption is completed.\n\n");

    /* Reset tamper coreblock */
    TAMPER_CORE_RESET();
    TAMPER_CORE_RELEASE();

    /* Enable voltage glitch detection clock source and select sampling rate */
    TAMPER_ENABLE_HIRC48M();
    TAMPER_VG_SAMPLE_SEL(TAMPER_VG_192M_SAMPLE);

    /* Initialize a reference trim value according to the power level of the system */
    TAMPER_VG_TRIM_INIT();

    /* Enable voltage glitch positive/negative detection interrupt */
    TAMPER_EnableInt(TAMPER_INTEN_VGPIEN_Msk | TAMPER_INTEN_VGNIEN_Msk);

    /* Clear voltage glitch positive/negative interrupt flag */
    TAMPER_CLR_INT_STATUS(TAMPER_INTSTS_VGPEVIF_Msk | TAMPER_INTSTS_VGNEVIF_Msk);

    NVIC_EnableIRQ(TAMPER_IRQn);

    /* Enable to clear crypto function */
    TAMPER_ENABLE_CRYPTO();

    NVIC_EnableIRQ(CRPT_IRQn);
    AES_ENABLE_INT(CRPT);

    /*---------------------------------------
     *  AES-128 ECB mode encrypt
     *---------------------------------------*/
    AES_Open(CRPT, 0, 1, AES_MODE_ECB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
    AES_SetKey(CRPT, 0, au32MyAESKey, AES_KEY_SIZE_128);
    AES_SetInitVect(CRPT, 0, au32MyAESIV);
    AES_SetDMATransfer(CRPT, 0, (uint32_t)au8InputData, (uint32_t)au8OutputData, sizeof(au8InputData));

    g_AES_done = 0;
    /* Start AES Encrypt */
    AES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    /* Waiting for AES calculation */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!g_AES_done)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for AES encrypt done time-out!\n");
            break;
        }
    }

    if(g_AES_done)
    {
        printf("AES encrypt done.\n\n");
        DumpBuffHex(au8OutputData, sizeof(au8InputData));
    }

    while(1);
}
