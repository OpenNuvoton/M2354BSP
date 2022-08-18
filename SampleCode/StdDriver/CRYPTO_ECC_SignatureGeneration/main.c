/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show Crypto IP ECC P-192 ECDSA signature generation function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


static char g_SHA_msg[] = "608079423f12421de616b7493ebe551cf4d65b92";      /* SHA-1 hash                                    */
static char gD[] = "e14f37b3d1374ff8b03f41b9b3fdd2f0ebccf275d660d7f3";    /* private key                                   */
static char gK[] = "cb0abc7043a10783684556fb12c4154d57bc31a289685f25";    /* random integer k form [1, n-1]                */
static char gR[] = "6994d962bdd0d793ffddf855ec5bf2f91a9698b46258a63e";    /* Expected answer: R of (R,S) digital signature */
static char gS[] = "02ba6465a234903744ab02bc8521405b73cf5fc00e1a9f41";    /* Expected answer: S of (R,S) digital signature */

static char gR1[168], gS1[168]; /* temporary buffer used to keep digital signature (R,S) pair */

void CRPT_IRQHandler(void);
void  dump_buff_hex(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void DEBUG_PORT_Init(void);

void CRPT_IRQHandler(void)
{
    ECC_DriverISR(CRPT);
}


void  dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while(nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for(i = 0; i < 16; i++)
            printf("%02x ", pucBuff[nIdx + i]);
        printf("  ");
        for(i = 0; i < 16; i++)
        {
            if((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
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

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    NVIC_EnableIRQ(CRPT_IRQn);
    ECC_ENABLE_INT(CRPT);

    printf("+---------------------------------------------+\n");
    printf("|   Crypto ECC Public Key Generation Demo     |\n");
    printf("+---------------------------------------------+\n");

    /* Calculate ECC signature */
    if(ECC_GenerateSignature(CRPT, CURVE_P_192, g_SHA_msg, gD, gK, gR1, gS1) < 0)
    {
        printf("ECC signature generation failed!!\n");
        goto lexit;
    }

    /* Verify the signature R */
    if(memcmp(gR1, gR, sizeof(gR)))
    {
        printf("Signature R [%s] is not matched with expected [%s]!\n", gR1, gR);
        goto lexit;
    }

    /* Verify the signature S */
    if(memcmp(gS1, gS, sizeof(gS)))
    {
        printf("Signature S [%s] is not matched with expected [%s]!\n", gS1, gS);
        goto lexit;
    }

    printf("ECC digital signature compared OK.\n");

lexit:

    while(1);
}



