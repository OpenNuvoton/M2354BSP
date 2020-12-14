/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * @brief    USB CCID Smartcard Reader sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "ccid.h"
#include "ccid_if.h"
#include "sclib.h"


#define CRYSTAL_LESS        1


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define INT_BUFFER_SIZE     64    /* Interrupt message buffer size */
#define BULK_BUFFER_SIZE    512   /* bulk message buffer size */

uint8_t UsbIntMessageBuffer[INT_BUFFER_SIZE];
uint8_t UsbMessageBuffer[BULK_BUFFER_SIZE];

uint8_t volatile gu8IsDeviceReady;
uint8_t volatile gu8AbortRequestFlag;
uint8_t volatile gu8IsBulkOutReady;
uint8_t volatile gu8IsBulkInReady;

uint8_t *pu8IntInBuf;
uint8_t *pUsbMessageBuffer;
uint32_t volatile u32BulkSize;

int32_t volatile gi32UsbdMessageLength;

void SC0_IRQHandler(void);
void UART_Init(void);
void SYS_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of smartcard port 0                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void SC0_IRQHandler(void)
{
    /* Please don't remove any of the function calls below */
    if(SCLIB_CheckCDEvent(0))
    {
        RDR_to_PC_NotifySlotChange();
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), pu8IntInBuf, 2);
        USBD_SET_PAYLOAD_LEN(EP4, 2);
        return; // Card insert/remove event occurred, no need to check other event...
    }

    SCLIB_CheckTimeOutEvent(0);
    SCLIB_CheckTxRxEvent(0);
    SCLIB_CheckErrorEvent(0);

    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
#if (!CRYSTAL_LESS)
    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_96MHz_HXT;
#else
    /* Enable internal HIRC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_96MHz_HIRC;
#endif

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set UART0 module clock */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure relative smartcard reader settings                                                            */
    /*---------------------------------------------------------------------------------------------------------*/
#if (!CRYSTAL_LESS)
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HXT, CLK_CLKDIV1_SC0(3));
#else
    /* M2354 HIRC is 12MHz */
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HIRC, CLK_CLKDIV1_SC0(3));
#endif
    CLK_EnableModuleClock(SC0_MODULE);

    /* Set SC0 multi-function pin */
    SYS->GPB_MFPL &= ~(SC0_PWR_PB2_Msk | SC0_RST_PB3_Msk | SC0_CLK_PB5_Msk | SC0_DAT_PB4_Msk);
    SYS->GPB_MFPL |= (SC0_PWR_PB2 | SC0_RST_PB3 | SC0_CLK_PB5 | SC0_DAT_PB4);
    SYS->GPC_MFPH &= ~(SC0_nCD_PC12_Msk);
    SYS->GPC_MFPH |= (SC0_nCD_PC12);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure relative USBD settings                                                                        */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select USBD function */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk | SYS_USBPHY_SBO_Msk;

    /* Set and enable USBD module clock */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(2));
    CLK_EnableModuleClock(USBD_MODULE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------+\n");
    printf("|    NuMicro USB CCID Smartcard Reader Sample Code    |\n");
    printf("+-----------------------------------------------------+\n\n");

    printf("# Smartcard reader I/O configuration:\n");
    printf("    SC0PWR (PB.2)  <--> smart card slot power pin\n");
    printf("    SC0RST (PB.3)  <--> smart card slot reset pin\n");
    printf("    SC0CLK (PB.5)  <--> smart card slot clock pin\n");
    printf("    SC0DAT (PB.4)  <--> smart card slot data pin\n");
    printf("    SC0CD  (PC.12) <--> smart card slot card detect pin\n\n");

    /* Open smartcard interface 0. CD pin state low indicates card absent and PWR pin low raise VCC pin to card */
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
    SC_ENABLE_INT(SC0, SC_INTEN_CDIEN_Msk);
    NVIC_EnableIRQ(SC0_IRQn);

    /* Open USB function */
    USBD_Open(&gsInfo, CCID_ClassRequest, NULL);

    /* Endpoint configuration */
    CCID_Init();

    /* Set priority is a must under current architecture. Otherwise smartcard interrupt will be blocked by USBD interrupt */
    NVIC_SetPriority(USBD_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();

    while(1) {}
}
