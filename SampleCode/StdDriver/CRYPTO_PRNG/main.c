/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Generate random numbers using Crypto IP PRNG
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define GENERATE_COUNT      10


static volatile int  g_PRNG_done;

void CRPT_IRQHandler(void);
void SYS_Init(void);
void DEBUG_PORT_Init(void);


void CRPT_IRQHandler(void)
{
    if(PRNG_GET_INT_FLAG(CRPT))
    {
        g_PRNG_done = 1;
        PRNG_CLR_INT_FLAG(CRPT);
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

    /* Enable CRYPTO module clock */
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


void DEBUG_PORT_Init(void)
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
    uint32_t    i, u32KeySize;
    uint32_t    au32PrngData[8];
    uint32_t u32TimeOutCnt;

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+-----------------------------------+\n");
    printf("|       Crypto PRNG Sample Demo     |\n");
    printf("+-----------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    PRNG_ENABLE_INT(CRPT);

    for(u32KeySize = PRNG_KEY_SIZE_128; u32KeySize <= PRNG_KEY_SIZE_256; u32KeySize++)
    {
        printf("\n\nPRNG Key size = %s\n\n",
               (u32KeySize == PRNG_KEY_SIZE_128) ? "128" :
               (u32KeySize == PRNG_KEY_SIZE_192) ? "192" :
               (u32KeySize == PRNG_KEY_SIZE_256) ? "256" : "unknown");

        /* start PRNG with seed 0x55 */
        PRNG_Open(CRPT, u32KeySize, 1, 0x55);

        for(i = 0; i < GENERATE_COUNT; i++)
        {
            g_PRNG_done = 0;
            /* Start random number generator */
            PRNG_Start(CRPT);

            /* Waiting for number ready */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!g_PRNG_done)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for PRNG time-out!\n");
                    goto lexit;
                }
            }

            /* Read random number */
            memset(au32PrngData, 0, sizeof(au32PrngData));
            PRNG_Read(CRPT, au32PrngData);

            printf("PRNG DATA ==>\n");
            printf("    0x%08x  0x%08x  0x%08x  0x%08x\n", au32PrngData[0], au32PrngData[1], au32PrngData[2], au32PrngData[3]);
            printf("    0x%08x  0x%08x  0x%08x  0x%08x\n", au32PrngData[4], au32PrngData[5], au32PrngData[6], au32PrngData[7]);
        }
    }

    printf("\nAll done.\n");

lexit:

    while(1);
}



