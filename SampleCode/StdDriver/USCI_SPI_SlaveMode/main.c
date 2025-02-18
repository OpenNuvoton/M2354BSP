/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Configure USCI_SPI1 as Slave mode and demonstrate how to communicate with an off-chip SPI Master device.
 *           This sample code needs to work with USCI_SPI_MasterMode sample code.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT  16

static uint32_t s_au32SourceData[TEST_COUNT];
static uint32_t s_au32DestinationData[TEST_COUNT];

/* Function prototype declaration */
void SYS_Init(void);
void USCI_SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main()
{
    uint32_t u32TxDataCount, u32RxDataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init USCI_SPI1 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------+\n");
    printf("|           USCI_SPI Slave Mode Sample Code           |\n");
    printf("+-----------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI1 as a slave.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI1:\n");
    printf("    USCI_SPI1_SS (PB.5)\n    USCI_SPI1_CLK (PB.1)\n");
    printf("    USCI_SPI1_MISO (PB.3)\n    USCI_SPI1_MOSI (PB.2)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for(u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        s_au32SourceData[u32TxDataCount] = 0xAA00 + u32TxDataCount;
        /* Clear destination buffer */
        s_au32DestinationData[u32TxDataCount] = 0;
    }

    u32TxDataCount = 0;
    u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.");
    getchar();
    printf("\n");

    /* Access TX and RX Buffer */
    while(u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if((USPI_GET_TX_FULL_FLAG(USPI1) == 0) && (u32TxDataCount < TEST_COUNT))
            USPI_WRITE_TX(USPI1, s_au32SourceData[u32TxDataCount++]); /* Write to TX Buffer */
        /* Check RX EMPTY flag */
        if(USPI_GET_RX_EMPTY_FLAG(USPI1) == 0)
            s_au32DestinationData[u32RxDataCount++] = USPI_READ_RX(USPI1); /* Read RX Buffer */
    }

    /* Print the received data */
    printf("Received data:\n");
    for(u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        printf("%d:\t0x%X\n", u32RxDataCount, s_au32DestinationData[u32RxDataCount]);
    }
    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Close USCI_SPI1 */
    USPI_Close(USPI1);
    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable USCI1 peripheral clock */
    CLK_EnableModuleClock(USCI1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set USCI1_SPI multi-function pins */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB1MFP_USCI1_CLK | SYS_GPB_MFPL_PB2MFP_USCI1_DAT0 | SYS_GPB_MFPL_PB3MFP_USCI1_DAT1 | SYS_GPB_MFPL_PB5MFP_USCI1_CTL0);
}

void USCI_SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI_SPI1                                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI1 as a slave, USCI_SPI1 clock rate = f_PCLK1,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI_Open(USPI1, USPI_SLAVE, USPI_MODE_0, 16, (uint32_t)NULL);
    /* Configure USCI_SPI_SS pin as low-active. */
    USPI1->CTLIN0 = (USPI1->CTLIN0 & ~USPI_CTLIN0_ININV_Msk) | USPI_CTLIN0_ININV_Msk;
}
