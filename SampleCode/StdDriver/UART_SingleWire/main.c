/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data by UART Single-Wire mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"


#define BUFSIZE     128


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_u8RecData[BUFSIZE] = {0};
static uint8_t g_u8TxData [BUFSIZE] = {0};
static volatile uint32_t g_u32RecLen  =  0;
static volatile int32_t  g_i32RecOK  = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART1_TEST_HANDLE(void);
void UART2_TEST_HANDLE(void);
void UART_FunctionTest(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void UART2_Init(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void Build_Src_Pattern(uint32_t u32Addr, uint8_t type, uint32_t u32Length);
uint8_t Check_Pattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length);


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

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(UART2_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL2_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL2_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set PB multi-function pin for UART1 RXD(PB.6) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk)) | SYS_GPB_MFPL_PB6MFP_UART1_RXD;

    /* Set PB multi-function pin for UART2 RXD(PB.0) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART2_RXD;

    /* The RX pin needs to pull-high for single-wire */
    /* If the external circuit doesn't pull-high, set GPIO pin as Quasi-directional mode for this purpose here */
    GPIO_SetMode(PB, BIT0, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, BIT6, GPIO_MODE_QUASI);

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

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure Single Wire(UART1) and set Single Wire(UART1) baud rate */
    UART_Open(UART1, 115200);
    UART_SelectSingleWireMode(UART1);
}

void UART2_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Reset UART2 */
    SYS_ResetModule(UART2_RST);

    /* Configure Single Wire(UART2) and set Single Wire(UART2) baud rate */
    UART_Open(UART2, 115200);
    UART_SelectSingleWireMode(UART2);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* Debug port control the Single wire 1(UART1) send data to Single wire 2(UART2)                           */
/* or Single wire 2(UART2) send data to Single wire 1(UART1)                                               */
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

    /* Init UART1 and UART2 for Single Wire Test */
    UART1_Init();
    UART2_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                           SAMPLE CODE                                                   */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("\nUART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                       ISR to handle UART Channel 1 interrupt event                                      */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART1_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                  UART1 Callback function                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE()
{
    uint32_t u32temp;

    /* Handle Receive Data Available interrupt */
    if(UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAIF_Msk))
    {
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART1))
        {
            /* Get the character from UART Buffer */
            u32temp = UART_READ(UART1);
            g_u8RecData[g_u32RecLen] = (uint8_t)u32temp;

            if(g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    /* Handle Single-wire Bit Error Detection interrupt */
    if(UART_GET_INT_FLAG(UART1, UART_INTSTS_SWBEIF_Msk))
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART1, UART_INTSTS_SWBEINT_Msk);
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 2 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_IRQHandler(void)
{
    UART2_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART1 Callback function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_TEST_HANDLE()
{
    uint32_t u32temp;

    /* Handle Receive Data Available interrupt */
    if(UART_GET_INT_FLAG(UART2, UART_INTSTS_RDAIF_Msk))
    {
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART2))
        {
            /* Get the character from UART Buffer */
            u32temp = UART_READ(UART2);
            g_u8RecData[g_u32RecLen] = (uint8_t)u32temp;

            if(g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    /* Handle Single-wire Bit Error Detection interrupt */
    if(UART_GET_INT_FLAG(UART2, UART_INTSTS_SWBEIF_Msk))
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART2, UART_INTSTS_SWBEINT_Msk);

    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*                              Build Source Pattern function                                              */
/*---------------------------------------------------------------------------------------------------------*/
void Build_Src_Pattern(uint32_t u32Addr, uint8_t type, uint32_t u32Length)
{
    uint32_t i = 0, pattern = 0;
    uint8_t *pAddr;
    pAddr = (uint8_t *)u32Addr;

    if(type == 0)      pattern = 0x1f;
    else if(type == 1) pattern = 0x3f;
    else if(type == 2) pattern = 0x7f;
    else if(type == 3) pattern = 0xff;
    else  pattern = 0xff;

    for(i = 0; i < u32Length ; i++)
        pAddr[i] = (i & (uint8_t)pattern);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                    Verify that the received data is correct                                             */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t Check_Pattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length)
{
    uint32_t i = 0;
    uint8_t result = 1;
    uint8_t *pAddr0;
    uint8_t *pAddr1;
    pAddr0 = (uint8_t *)u32Addr0;
    pAddr1 = (uint8_t *)u32Addr1;

    for(i = 0; i < u32Length ; i++)
    {
        if(pAddr0[i] != pAddr1[i])
        {
            printf("Data Error Index=%d,tx =%d,rx=%d\n", i, pAddr0[i], pAddr1[i]) ;
            result = 0;
        }
    }

    return result;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest()
{
    uint8_t u8Cmd;
    uint32_t u32TimeOutCnt;

    printf("+------------------------------------------------------------+\n");
    printf("|            UART Single Wire Function Test                  |\n");
    printf("+------------------------------------------------------------+\n");
    printf("|  Description :                                             |\n");
    printf("|    This sample code shows data transmission by UART        |\n");
    printf("|    Single-Wire mode. User must connect UART1_RX pin(PB6)   |\n");
    printf("|    to UART2_Rx Pin(PB0).                                   |\n");
    printf("+------------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART0 and PC. UART0 is set to debug port.
        UART1 and UART2 are set to UART single wire mode for data transmission.
        User can use UART0 to control the transmission or reception of UART1(Single Wire mode).
        When UART1(Single Wire 1) transfers data to UART2(Single Wire 2), if data is valid,
        it will enter the interrupt and receive the data. And then check the received data.
        When UART2(Single Wire 2) transfers data to UART1(Single Wire 1), if data is valid,
        it will enter the interrupt and receive the data. And then check the received data.
    */

    /* Enable UART1 RDA/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_SWBEIEN_Msk));

    /* Enable UART2 RDA/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART2_IRQn);
    UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_SWBEIEN_Msk));

    do
    {
        printf("+------------------------------------------------------------+\n");
        printf("|                UART Single Wire Test Item                  |\n");
        printf("+------------------------------------------------------------+\n");
        printf("| Please select test item:                                   |\n");
        printf("|   (1)Single Wire 1(PB6) send data to Single Wire 2(PB0).   |\n");
        printf("|   (2)Single Wire 2(PB0) send data to Single Wire 1(PB6).   |\n");
        printf("|   (E)Exit                                                  |\n");
        printf("+------------------------------------------------------------+\n");

        u8Cmd = (uint8_t)getchar();

        switch(u8Cmd)
        {
            case '1':
            {
                printf("SW1(UART1) --> SW2(UART2)Test :");
                g_i32RecOK  = FALSE;
                Build_Src_Pattern((uint32_t)g_u8TxData, UART_WORD_LEN_8, BUFSIZE);

                /* Check the Rx status is Idle */
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
                while(!UART_RX_IDLE(UART1))
                {
                    if(--u32TimeOutCnt == 0)
                    {
                        printf("Wait for UART Rx idle time-out!\n");
                        return;
                    }
                }
                UART_Write(UART1, g_u8TxData, BUFSIZE);
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
                while(g_i32RecOK != TRUE)
                {
                    if(--u32TimeOutCnt == 0)
                    {
                        printf("Wait for UART Rx data time-out!\n");
                        return;
                    }
                }

                Check_Pattern((uint32_t)g_u8TxData, (uint32_t)g_u8RecData, BUFSIZE) ? printf(" Pass\n") : printf(" Fail\n");
                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_u8TxData, 0, BUFSIZE);
                memset((uint8_t *)g_u8RecData, 0, BUFSIZE);
            }
            break;

            case '2':
            {
                printf("SW2(UART2) --> SW1(UART1)Test :");
                g_i32RecOK  = FALSE;
                Build_Src_Pattern((uint32_t)g_u8TxData, UART_WORD_LEN_8, BUFSIZE);

                /* Check the Rx status is Idle */
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
                while(!UART_RX_IDLE(UART2))
                {
                    if(--u32TimeOutCnt == 0)
                    {
                        printf("Wait for UART Rx idle time-out!\n");
                        return;
                    }
                }
                UART_Write(UART2, g_u8TxData, BUFSIZE);
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
                while(g_i32RecOK != TRUE)
                {
                    if(--u32TimeOutCnt == 0)
                    {
                        printf("Wait for UART Rx data time-out!\n");
                        return;
                    }
                }

                Check_Pattern((uint32_t)g_u8TxData, (uint32_t)g_u8RecData, BUFSIZE) ? printf(" Pass\n") :   printf(" Fail\n");

                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_u8TxData, 0, BUFSIZE);
                memset((uint8_t *)g_u8RecData, 0, BUFSIZE);
            }
            break;

            default:
                break;
        }

    }
    while((u8Cmd != 'E') && (u8Cmd != 'e'));

    /* Disable UART1 RDA/Single-wire Bit Error Detection interrupt */
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    NVIC_DisableIRQ(UART1_IRQn);

    /* Disable UART2 RDA/Single-wire Bit Error Detection interrupt */
    UART_DisableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    NVIC_DisableIRQ(UART1_IRQn);
    printf("\nUART Sample Demo End.\n");

}
