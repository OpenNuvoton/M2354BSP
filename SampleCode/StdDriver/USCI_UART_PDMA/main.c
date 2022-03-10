/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive UART data with PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



#define UUART_RX_DMA_CH 0
#define UUART_TX_DMA_CH 1


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static int32_t g_i32UuartTestLength = 64;
static uint8_t g_au8SrcArray[64];
static uint8_t g_au8DestArray[64];
static volatile int32_t g_i32IntCnt;
static volatile int32_t g_i32IsTestOver;
static volatile uint32_t g_u32TwoChannelPdmaTest = 0;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void ClearBuf(uint32_t u32Addr, uint32_t u32Length, uint8_t u8Pattern);
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length);
void PDMA_UUART_TxTest(void);
void PDMA_UUART_RxTest(void);
void PDMA_Callback_0(void);
void PDMA_Callback_1(void);
void PDMA0_IRQHandler(void);
void UART0_IRQHandler(void);
void PDMA_UART(int32_t i32option);
void SYS_Init(void);
void UART0_Init(void);
void USCI0_Init(void);
/*---------------------------------------------------------------------------------------------------------*/
/* Clear buffer function                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void ClearBuf(uint32_t u32Addr, uint32_t u32Length, uint8_t u8Pattern)
{
    uint8_t* pu8Ptr;
    uint32_t u32Idx;

    pu8Ptr = (uint8_t *)u32Addr;

    for(u32Idx = 0; u32Idx < u32Length; u32Idx++)
    {
        *pu8Ptr++ = u8Pattern;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Build Src Pattern function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length)
{
    uint32_t u32Idxi = 0, u32Idxj, loop;
    uint8_t* pAddr;

    pAddr = (uint8_t *)u32Addr;

    do
    {
        if(u32Length > 256)
            loop = 256;
        else
            loop = u32Length;

        u32Length = u32Length - loop;

        for(u32Idxj = 0; u32Idxj < loop; u32Idxj++)
            *pAddr++ = (uint8_t)(u32Idxj + u32Idxi);

        u32Idxi++;
    }
    while((loop != 0) || (u32Length != 0));
}

/*---------------------------------------------------------------------------------------------------------*/
/* UUART Tx PDMA Channel Configuration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UUART_TxTest(void)
{
    /* UUART Tx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, UUART_TX_DMA_CH, PDMA_WIDTH_8, (uint32_t)g_i32UuartTestLength);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, UUART_TX_DMA_CH, (uint32_t)g_au8SrcArray, PDMA_SAR_INC, (uint32_t)&UUART0->TXDAT, PDMA_DAR_FIX);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, UUART_TX_DMA_CH, PDMA_USCI0_TX, FALSE, 0);

    /* Single request type */
    PDMA_SetBurstType(PDMA0, UUART_TX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    PDMA0->DSCT[UUART_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* UUART Rx PDMA Channel Configuration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UUART_RxTest(void)
{
    /* UUART Rx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, UUART_RX_DMA_CH, PDMA_WIDTH_8, (uint32_t)g_i32UuartTestLength);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, UUART_RX_DMA_CH, (uint32_t)&UUART0->RXDAT, PDMA_SAR_FIX, (uint32_t)g_au8DestArray, PDMA_DAR_INC);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, UUART_RX_DMA_CH, PDMA_USCI0_RX, FALSE, 0);

    /* Single request type */
    PDMA_SetBurstType(PDMA0, UUART_RX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    PDMA0->DSCT[UUART_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_Callback_0(void)
{
    printf("\tTransfer Done %d!\r", ++g_i32IntCnt);

    /* Use PDMA to do UART loopback test 10 times */
    if(g_i32IntCnt < 10)
    {
        /* UUART Tx and Rx PDMA configuration */
        PDMA_UUART_TxTest();
        PDMA_UUART_RxTest();

        /* Enable UUART Tx and Rx PDMA function */
        UUART_PDMA_ENABLE(UUART0, UUART_PDMACTL_PDMAEN_Msk | UUART_PDMACTL_TXPDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk);
    }
    else
    {
        /* Test is over */
        g_i32IsTestOver = TRUE;
    }
}

void PDMA_Callback_1(void)
{
    int32_t i32Idx;
    uint8_t u8InChar = 0xFF;

    printf("\tTransfer Done %d!\t", ++g_i32IntCnt);

    /* Show UART Rx data */
    for(i32Idx = 0; i32Idx < g_i32UuartTestLength; i32Idx++)
    {
        u8InChar = inpb(((uint32_t)g_au8DestArray + (uint32_t)i32Idx));
        printf(" 0x%x(%c),", u8InChar, u8InChar);
    }
    printf("\n");

    /* Use PDMA to do UART Rx test 10 times */
    if(g_i32IntCnt < 10)
    {
        /* UUART Rx PDMA configuration */
        PDMA_UUART_RxTest();

        /* Enable UUART Rx PDMA function */
        UUART_PDMA_ENABLE(UUART0, UUART_PDMACTL_PDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk);
    }
    else
    {
        /* Test is over */
        g_i32IsTestOver = TRUE;
    }
}

void PDMA0_IRQHandler(void)
{
    volatile uint32_t u32temp;

    /* Get PDMA interrupt status */
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if(status & PDMA_INTSTS_ABTIF_Msk)  /* Target Abort */
    {
        g_i32IsTestOver = 2;
        u32temp = PDMA0->ABTSTS;
        PDMA0->ABTSTS = u32temp;
    }
    else if(status & PDMA_INTSTS_TDIF_Msk) /* Transfer Done */
    {
        /* UART Tx PDMA transfer done interrupt flag */
        if(PDMA_GET_TD_STS(PDMA0) & (1 << UUART_TX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA0, (1 << UUART_TX_DMA_CH));

            /* Disable UUART Tx PDMA function */
            UUART_PDMA_DISABLE(UUART0, UUART_PDMACTL_TXPDMAEN_Msk);
        }

        /* UART Rx PDMA transfer done interrupt flag */
        if(PDMA_GET_TD_STS(PDMA0) & (1 << UUART_RX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA0, (1 << UUART_RX_DMA_CH));

            /* Disable UUART Rx PDMA function */
            UUART_PDMA_DISABLE(UUART0, UUART_PDMACTL_RXPDMAEN_Msk);

            /* Handle PDMA transfer done interrupt event */
            if(g_u32TwoChannelPdmaTest == 1)
            {
                PDMA_Callback_0();
            }
            else if(g_u32TwoChannelPdmaTest == 0)
            {
                PDMA_Callback_1();
            }
        }
    }
    else
    {
        printf("unknown interrupt, status=0x%x !!\n", status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 5 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    /* Get UART0 Rx data and send the data to UUART0 Tx */
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAIF_Msk))
        UUART0->TXDAT = UART0->DAT;
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Sample Code:                                                                                       */
/*         i32option : ['1'] UUART0 TX/RX PDMA Loopback                                                    */
/*                     [Others] UUART0 RX PDMA test                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART(int32_t i32option)
{
    uint32_t u32TimeOutCnt;

    /* Source data initiation */
    BuildSrcPattern((uint32_t)g_au8SrcArray, (uint32_t)g_i32UuartTestLength);
    ClearBuf((uint32_t)g_au8DestArray, (uint32_t)g_i32UuartTestLength, 0xFF);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    if(i32option == '1')
    {
        printf("  [Using TWO PDMA channel].\n");
        printf("  This sample code will use PDMA0 to do USCI0 loopback test 10 times.\n");
        printf("  Please connect USCI0_DAT0(PE.3) <--> USCI0_DAT1(PE.4) before testing.\n");
        printf("  After connecting PE.3 <--> PE.4, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 1;
        getchar();
    }
    else
    {
        g_i32UuartTestLength = 2;      /* Test Length */
        printf("  [Using ONE PDMA channel].\n");
        printf("  This sample code will use PDMA0 to do USCI0 Rx test 10 times.\n");
        printf("  Please connect USCI0_DAT0(PE.3) <--> USCI0_DAT1(PE.4) before testing.\n");
        printf("  After connecting PE.3 <--> PE.4, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 0;
        getchar();
        printf("  Please input %d bytes to trigger PDMA one time.(Ex: Press 'a''b')\n", g_i32UuartTestLength);
    }

    if(g_u32TwoChannelPdmaTest == 1)
    {
        /* Enable PDMA channel */
        PDMA_Open(PDMA0, (1 << UUART_RX_DMA_CH) | (1 << UUART_TX_DMA_CH));

        /* UART Tx and Rx PDMA configuration */
        PDMA_UUART_TxTest();
        PDMA_UUART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA_EnableInt(PDMA0, UUART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
        PDMA_EnableInt(PDMA0, UUART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    }
    else
    {
        /* Enable PDMA channel */
        PDMA_Open(PDMA0, (1 << UUART_RX_DMA_CH));

        /* UUART Rx PDMA configuration */
        PDMA_UUART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA_EnableInt(PDMA0, UUART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    }

    /* Enable PDMA Transfer Done Interrupt */
    g_i32IntCnt = 0;
    g_i32IsTestOver = FALSE;
    NVIC_EnableIRQ(PDMA0_IRQn);

    /* Enable UART0 RDA interrupt */
    if(g_u32TwoChannelPdmaTest == 0)
    {
        NVIC_EnableIRQ(UART0_IRQn);
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
    }

    /* Enable UUART Tx and Rx PDMA function */
    if(g_u32TwoChannelPdmaTest == 1)
        UUART_PDMA_ENABLE(UUART0, UUART_PDMACTL_PDMAEN_Msk | UUART_PDMACTL_TXPDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk);
    else
        UUART_PDMA_DISABLE(UUART1, UUART_PDMACTL_TXPDMAEN_Msk);

    UUART_PDMA_ENABLE(UUART0, UUART_PDMACTL_PDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk);

    /* Wait for PDMA operation finish */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_i32IsTestOver == FALSE)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA operation finish time-out!\n");
            break;
        }
    }

    /* Check PDMA status */
    if(g_i32IsTestOver == 2)
        printf("target abort...\n");

    /* Disable UUART Tx and Rx PDMA function */
    UUART_PDMA_DISABLE(UUART0, UUART_PDMACTL_PDMAEN_Msk | UUART_PDMACTL_TXPDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk);

    /* Disable PDMA channel */
    PDMA_Close(PDMA0);

    /* Disable PDMA Interrupt */
    PDMA_DisableInt(PDMA0, UUART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_DisableInt(PDMA0, UUART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Disable UART0 RDA interrupt */
    NVIC_DisableIRQ(UART0_IRQn);
    UART_DisableInt(UART0, UART_INTEN_RDAIEN_Msk);
}


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

    /* Enable UART, USCI and PDMA module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);
    CLK_EnableModuleClock(PDMA0_MODULE);

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
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    uint8_t u8Item;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init USCI0 */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUSCI UART PDMA Sample Program\n");

    /* USCI UART PDMA sample function */
    do
    {
        printf("\n\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|                   USCI UART PDMA Driver Sample Code                    |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("| [1] Using TWO PDMA channel to test. < TX1(CH1)-->RX1(CH0) >            |\n");
        printf("| [2] Using ONE PDMA channel to test. < TX1-->RX1(CH0) >                 |\n");
        printf("+------------------------------------------------------------------------+\n");
        u8Item = (uint8_t)getchar();

        g_i32IsTestOver = FALSE;
        if((u8Item == '1') || (u8Item == '2'))
        {
            PDMA_UART(u8Item);
            printf("\n\n  USCI UART PDMA sample code is complete.\n");
        }

    }
    while(u8Item != 27);

    printf("\nUSCI UART PDMA Sample Program End\n");

    while(1);

}
