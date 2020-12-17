/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data from PC terminal through RS232 interface.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_au8RecData[RXBUFSIZE]  = {0};

static volatile uint32_t g_u32comRbytes = 0;
static volatile uint32_t g_u32comRhead  = 0;
static volatile uint32_t g_u32comRtail  = 0;
static volatile int32_t g_i32Wait       = TRUE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void USCI_UART_TEST_HANDLE(void);
void USCI_UART_FunctionTest(void);
void SYS_Init(void);
void UART0_Init(void);
void USCI0_Init(void);
void USCI0_IRQHandler(void);


void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable UART0 and USCI0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set PE multi-function pins for USCI0_DAT0(PE.3) and USCI0_DAT1(PE.4) */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE3MFP_Msk)) | SYS_GPE_MFPL_PE3MFP_USCI0_DAT0;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE4MFP_Msk)) | SYS_GPE_MFPL_PE4MFP_USCI0_DAT1;

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

void USCI0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS_ResetModule(USCI0_RST);

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* USCI UART Test Sample                                                                                   */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

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

    /* Init USCI0 for test */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART sample function */
    USCI_UART_FunctionTest();

    printf("\nUSCI UART Sample Demo End.\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI UART interrupt event                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    USCI_UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* USCI UART Callback function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_TEST_HANDLE(void)
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UUART0->PROTSTS;

    if(u32IntSts & UUART_PROTSTS_RXENDIF_Msk)
    {

        /* Clear RX end interrupt flag */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);

        printf("\nInput:");

        /* Get all the input characters */
        while(!UUART_IS_RX_EMPTY(UUART0))
        {

            /* Get the character from USCI UART Buffer */
            u8InChar = (uint8_t)UUART_READ(UUART0);

            printf("%c ", u8InChar);

            if(u8InChar == '0')
            {
                g_i32Wait = FALSE;
            }

            /* Check if buffer full */
            if(g_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_au8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                g_u32comRbytes++;
            }
        }

        printf("\nTransmission Test:");
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI UART Function Test                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_FunctionTest(void)
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32Temp;

    printf("+-----------------------------------------------------------+\n");
    printf("|  USCI UART Function Test                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect USCI-UART0 and PC.
        USCI-UART0 is set to debug port. USCI-UART0 is enable RX and TX end interrupt.
        When inputting char to terminal screen, RX end interrupt will happen and
        USCI-UART0 will print the received char on screen.
    */

    /* Enable USCI UART interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_EnableIRQ(USCI0_IRQn);

    while(g_i32Wait)
    {
        u32Temp = g_u32comRtail;

        if(g_u32comRhead != u32Temp)
        {
            u8InChar = g_au8RecData[g_u32comRhead];

            while(UUART_IS_TX_FULL(UUART0));  /* Wait Tx is not full to transmit data */

            UUART_WRITE(UUART0, u8InChar);
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            g_u32comRbytes--;
        }
    }

    /* Disable USCI UART receive and transmit end interrupt */
    UUART_DISABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk | UUART_INTEN_TXENDIEN_Msk);
    NVIC_DisableIRQ(USCI0_IRQn);
    g_i32Wait = TRUE;

}
