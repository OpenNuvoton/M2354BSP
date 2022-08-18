/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show Crypto SM4 ECB mode encrypt/decrypt function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


void DEBUG_PORT_Init(void);
void CRPT_IRQHandler(void);
void SYS_Init(void);
void DumpBuffHex(const uint8_t *pucBuff, int nBytes);


static uint32_t au32MyAESKey[8] = {0x01234567, 0x89abcdef, 0xfedcba98, 0x76543210};


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
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10
};


__attribute__((aligned(4))) static uint8_t g_au8Out[] = {0};


static const uint8_t g_au8Golden1[] = {0x68, 0x1e, 0xdf, 0x34, 0xd2, 0x06, 0x96, 0x5e, 0x86, 0xb3, 0xe9, 0x4f, 0x53, 0x6e, 0x42, 0x46};
static const uint8_t g_au8Golden2[] = {0x59, 0x52, 0x98, 0xc7, 0xc6, 0xfd, 0x27, 0x1f, 0x04, 0x02, 0xf8, 0x04, 0xc3, 0x3d, 0x3f, 0x66};




#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8OutputData[1024];
#else
__attribute__((aligned(4))) static uint8_t au8OutputData[1024];
#endif

static volatile int32_t  g_AES_done;

void CRPT_IRQHandler()
{
    if(AES_GET_INT_FLAG(CRPT))
    {
        g_AES_done = 1;
        AES_CLR_INT_FLAG(CRPT);
    }
}


void DumpBuffHex(const uint8_t *pucBuff, int nBytes)
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

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable CRPT module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

}


void DEBUG_PORT_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    DEBUG_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i, i32Err;
    uint32_t u32TimeOutCnt;

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+---------------------------------------+\n");
    printf("|         Crypto SM4 Sample Code        |\n");
    printf("+---------------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    AES_ENABLE_INT(CRPT);

    printf("Source:\n");
    DumpBuffHex(au8InputData, sizeof(au8InputData));


    /*---------------------------------------
     *  Encrypt
     *---------------------------------------*/
    AES_Open(CRPT, 0, 1, SM4_MODE_ECB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
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
            goto lexit;
        }
    }

    printf("AES encrypt done.\nResult:\n");
    DumpBuffHex(au8OutputData, sizeof(au8InputData));

    /* Compare with golden pattern 1 */
    for(i = 0; i < 16; i++)
    {
        if(au8OutputData[i] != g_au8Golden1[i])
        {
            printf("The ecrypt output is different to golden pattern!\n");
            printf("Golden pattern:\n");
            DumpBuffHex(g_au8Golden1, sizeof(au8InputData));
            i32Err = -1;
            goto lexit;
        }
    }

    /*---------------------------------------
     *  Decrypt
     *---------------------------------------*/
    AES_Open(CRPT, 0, 0, SM4_MODE_ECB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
    AES_SetKey(CRPT, 0, au32MyAESKey, AES_KEY_SIZE_128);
    AES_SetInitVect(CRPT, 0, au32MyAESIV);
    AES_SetDMATransfer(CRPT, 0, (uint32_t)au8OutputData, (uint32_t)g_au8Out, sizeof(au8InputData));

    g_AES_done = 0;
    /* Start AES decrypt */
    AES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    /* Waiting for AES calculation */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!g_AES_done)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for AES decrypt done time-out!\n");
            goto lexit;
        }
    }

    printf("AES decrypt done.\nResult:\n");
    DumpBuffHex(g_au8Out, sizeof(g_au8Out));

    i32Err = 0;
    for(i = 0; i < (int32_t)sizeof(au8InputData); i++)
    {
        if(g_au8Out[i] != au8InputData[i])
        {
            i32Err = -1;
            goto lexit;
        }
    }


    //------------------------------------------------------------
    // Encrypt 1000000 times

    printf("Encrypt 1000000 times.\nRunning .");
    for(i = 0; i < 1000000; i++)
    {
        if((i & 0x3fff) == 0)
            printf(".");

        AES_Open(CRPT, 0, 1, SM4_MODE_ECB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
        AES_SetKey(CRPT, 0, au32MyAESKey, AES_KEY_SIZE_128);
        AES_SetInitVect(CRPT, 0, au32MyAESIV);

        AES_SetDMATransfer(CRPT, 0, (uint32_t)au8InputData, (uint32_t)au8InputData, sizeof(au8InputData));

        g_AES_done = 0;
        /* Start AES Encrypt */
        AES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
        /* Waiting for AES calculation */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(!g_AES_done)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for AES time-out!\n");
                goto lexit;
            }
        }
    }
    printf(" Done!\n");

    printf("AES decrypt done.\nResult:\n");
    DumpBuffHex(au8InputData, sizeof(au8InputData));

    i32Err = 0;
    for(i = 0; i < (int32_t)sizeof(au8InputData); i++)
    {
        if(g_au8Golden2[i] != au8InputData[i])
        {
            printf("The ecrypt output is different to golden pattern!\n");
            printf("Golden pattern:\n");
            DumpBuffHex(g_au8Golden2, sizeof(au8InputData));
            i32Err = -1;
            break;
        }
    }


    if(i32Err)
    {
        printf("TEST FAILED!\n");
    }
    else
    {
        printf("TEST PASSED\n");
    }

lexit:

    while(1);
}



