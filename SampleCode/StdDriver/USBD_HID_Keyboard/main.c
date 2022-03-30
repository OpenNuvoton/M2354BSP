/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to implement a USB keyboard device.
 *           It supports to use GPIO to simulate key input.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "hid_kb.h"

#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x10C)

/*--------------------------------------------------------------------------*/
uint8_t volatile g_u8EP2Ready = 0;

/*--------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void HID_UpdateKbData(void);
void PowerDown(void);
int IsDebugFifoEmpty(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

#if (!CRYSTAL_LESS)
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HXT;

    /* Waiting for PLL clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL and HCLK clock divider as 3 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(3));

    /* Select USB clock source as PLL and USB clock divider as 3 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(3));
#else
    /* Enable HIRC48 clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48EN_Msk);

    /* Waiting for HIRC48 clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48STB_Msk);

    /* Switch HCLK clock source to HIRC48 and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC48, CLK_CLKDIV0_HCLK(1));

    /* Select USB clock source as HIRC48 and USB clock divider as 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_HIRC48, CLK_CLKDIV0_USB(1));
#endif

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk | SYS_USBPHY_SBO_Msk;

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_USB_VBUS | SYS_GPA_MFPH_PA13MFP_USB_D_N | SYS_GPA_MFPH_PA14MFP_USB_D_P | SYS_GPA_MFPH_PA15MFP_USB_OTG_ID);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
}

void HID_UpdateKbData(void)
{
    int32_t i;
    uint8_t *pu8Buf;
    uint32_t u32Key = 0xF;
    static uint32_t u32PreKey;

    if(g_u8EP2Ready)
    {
        pu8Buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));

        /* If PF.11 = 0, just report it is key 'a' */
        u32Key = (PF->PIN & (1 << 11)) ? 0 : 1;

        if(u32Key == 0)
        {
            for(i = 0; i < 8; i++)
            {
                pu8Buf[i] = 0;
            }

            if(u32Key != u32PreKey)
            {
                /* Trigger to note key release */
                USBD_SET_PAYLOAD_LEN(EP2, 8);
            }
        }
        else
        {
            u32PreKey = u32Key;
            pu8Buf[2] = 0x04; /* Key a */
            USBD_SET_PAYLOAD_LEN(EP2, 8);
        }
    }
}

void PowerDown(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("Enter power down ...\n");
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!IsDebugFifoEmpty())
        if(--u32TimeOutCnt == 0) break;

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    printf("device wakeup!\n");

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|          NuMicro USB HID Keyboard Sample Code          |\n");
    printf("+--------------------------------------------------------+\n");
    printf("If PF.11 = 0 or press SW2 button, just report it is key 'a'.\n");

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();
    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    /* start to IN data */
    g_u8EP2Ready = 1;

    while(1)
    {
#if CRYSTAL_LESS
        /* Start USB trim if it is not enabled. */
        if((SYS->TCTL48M & SYS_TCTL48M_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->TCTL48M = 0x01;
                SYS->TCTL48M |= SYS_TCTL48M_REFCKSEL_Msk | SYS_TCTL48M_BOUNDEN_Msk | (8 << SYS_TCTL48M_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if(SYS->TISTS48M & (SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->TCTL48M = 0;

            /* Clear error flags */
            SYS->TISTS48M = SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif

        /* Enter power down when USB suspend */
        if(g_u8Suspend)
            PowerDown();

        HID_UpdateKbData();
    }
}
