/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show Crypto IP SHA function
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "vector_parser.h"

extern void OpenTestVector(void);
extern int  GetNextPattern(void);


static int32_t  g_i32DigestLength = 0;

static volatile int g_SHA_done;


void CRPT_IRQHandler(void);
int  do_compare(uint8_t *output, uint8_t *expect, int cmp_len);
int32_t RunSHA(void);
void SYS_Init(void);
void DEBUG_PORT_Init(void);


void CRPT_IRQHandler(void)
{
    if(SHA_GET_INT_FLAG(CRPT))
    {
        g_SHA_done = 1;
        SHA_CLR_INT_FLAG(CRPT);
    }
}


int  do_compare(uint8_t *output, uint8_t *expect, int cmp_len)
{
    int   i;

    if(memcmp(expect, output, (size_t)cmp_len))
    {
        printf("\nMismatch!! - %d\n", cmp_len);
        for(i = 0; i < cmp_len; i++)
            printf("0x%02x    0x%02x\n", expect[i], output[i]);
        return -1;
    }
    return 0;
}


int32_t RunSHA(void)
{
    uint32_t  au32OutputDigest[8];

    SHA_Open(CRPT, SHA_MODE_SHA1, SHA_IN_SWAP, 0);

    SHA_SetDMATransfer(CRPT, (uint32_t)&g_au8ShaData[0], (uint32_t)g_i32DataLen / 8);

    printf("Key len= %d bits\n", g_i32DataLen);

    g_SHA_done = 0;
    /* Start SHA calculation */
    SHA_Start(CRPT, CRYPTO_DMA_ONE_SHOT);
    
    /* Waiting for SHA calcuation done */
    while(!g_SHA_done) ;
    
    /* Read SHA calculation result */
    SHA_Read(CRPT, au32OutputDigest);

    /* Compare calculation result with golden pattern */
    if(do_compare((uint8_t *)&au32OutputDigest[0], &g_au8ShaDigest[0], g_i32DigestLength) < 0)
    {
        printf("Compare error!\n");
        while(1);
    }
    return 0;
}


void SYS_Init(void)
{
    /* Enable HIRC, HXT and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_HXTEN_Msk|CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk|CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_96MHZ);

    /* Select IP clock source */
    CLK->CLKSEL2 = CLK_CLKSEL2_UART0SEL_HIRC;
    CLK->CLKSEL3 = CLK_CLKSEL3_UART5SEL_HIRC;

    /* Enable IP clock */
    CLK->AHBCLK  |= CLK_AHBCLK_CRPTCKEN_Msk;
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_UART5CKEN_Msk;


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 96000000;            // PLL
    SystemCoreClock = 96000000 / 1;        // HCLK
    CyclesPerUs     = 96000000 / 1000000;  // For CLK_SysTickDelay()

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


/*-----------------------------------------------------------------------------*/
int main(void)
{

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+-----------------------------------+\n");
    printf("|       Crypto SHA Sample Demo      |\n");
    printf("+-----------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    SHA_ENABLE_INT(CRPT);

    /* Load test vector data base */
    OpenTestVector();

    while(1)
    {   
        /* Get data from test vector to calcualte and 
           compre the result with golden pattern */
        if(GetNextPattern() < 0)
            break;

        RunSHA();
    }

    printf("SHA test done.\n");

    while(1);
}
