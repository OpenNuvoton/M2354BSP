/**************************************************************************//**
 * @file     SecureISP.c
 * @version  V3.00
 * @brief    SecureISP initialization source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <arm_cmse.h>
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "hid_transfer.h"

#include "CommandHandler.h"

static volatile ISP_INFO_T  g_ISPInfo = {0};

extern int32_t Exec_VendorFunction(uint32_t *pu32Buf, uint32_t u32Len);

/* Declare Client's ECC Key pair */
static const char gacPrivKey[] = "d0ab2cb9eb88976e82f107598077ce50d8c7b67def7039ee5ba39ee0dd3be411";
static const char gacPubKey0[] = "d32438a1b4428541c564eeed79669b4bd3bf601c758469545e013c8fe8af7ef6";
static const char gacPubKey1[] = "476de8f3c6e6c48a8bacf1e1827cfb82501833c2bb816344f996533b1b031706";

void UART1_IRQHandler(void);
void CRPT_IRQHandler(void);
void SendUART1DataOut(uint8_t *pu8Buf);
int32_t ExecuteSecureISP(void);

/* To generate 1st ECDH shared key in AESKey */
static int32_t GenECDHSharedKey(ECC_PUBKEY_T *PubKey, uint32_t *AESKey)
{
    volatile int32_t    ret = 0;
    char                Qx[70], Qy[70], z[70];
    uint32_t            tmp[8];

    memset(Qx, 0x0, sizeof(Qx));
    memset(Qy, 0x0, sizeof(Qy));
    memset(z,  0x0, sizeof(z));

    /* Set Qx */
    memcpy(tmp, (char*)PubKey->au32Key0, sizeof(tmp));
    BytesSwap((char*)tmp, sizeof(tmp));
    CRPT_Reg2Hex(64, tmp, Qx);

    /* Set Qy */
    memcpy(tmp, (char*)PubKey->au32Key1, sizeof(tmp));
    BytesSwap((char*)tmp, sizeof(tmp));
    CRPT_Reg2Hex(64, tmp, Qy);

    printf("\nTo generate 1st shared key:\n");
    printf("d:  %s\n", (const char *)gacPrivKey);
    printf("Qx: %s\n", Qx);
    printf("Qy: %s\n", Qy);
    ret = ECC_GenerateSecretZ(CRPT, CURVE_P_256, (char *)((uint32_t)gacPrivKey), Qx, Qy, z);
    if(ret < 0)
    {
        memset(Qx, 0x0, sizeof(Qx));
        memset(Qy, 0x0, sizeof(Qy));
        memset(z,  0x0, sizeof(z));
        return -1;
    }

    printf("z:  %s\n", z);

    CRPT_Hex2Reg(z, AESKey);
    BytesSwap((char *)AESKey, (256 / 8));

    memset(Qx, 0x0, sizeof(Qx));
    memset(Qy, 0x0, sizeof(Qy));
    memset(z,  0x0, sizeof(z));

    printf("\n");
    return 0;
}

static void SetClientPublicKey(uint32_t *pub0, uint32_t *pub1)
{
    CRPT_Hex2Reg((char *)((uint32_t)gacPubKey0),  pub0);
    BytesSwap((char *)pub0, sizeof(pub0)); // 256-bits

    CRPT_Hex2Reg((char *)((uint32_t)gacPubKey1),  pub1);
    BytesSwap((char *)pub1, sizeof(pub1)); // 256-bits

    /* ECC key pair example */
//  "d32438a1b4428541c564eeed79669b4bd3bf601c758469545e013c8fe8af7ef6"
//    /* B0, B1, ... B63 */
//    pub0[0] = 0xa13824d3;
//    pub0[1] = 0x418542b4;
//    pub0[2] = 0xedee64c5;
//    pub0[3] = 0x4b9b6679;
//    pub0[4] = 0x1c60bfd3;
//    pub0[5] = 0x54698475;
//    pub0[6] = 0x8f3c015e;
//    pub0[7] = 0xf67eafe8;

//  "476de8f3c6e6c48a8bacf1e1827cfb82501833c2bb816344f996533b1b031706"
//    /* B0, B1, ... B63 */
//    pub1[0] = 0xf3e86d47;
//    pub1[1] = 0x8ac4e6c6;
//    pub1[2] = 0xe1f1ac8b;
//    pub1[3] = 0x82fb7c82;
//    pub1[4] = 0xc2331850;
//    pub1[5] = 0x446381bb;
//    pub1[6] = 0x3b5396f9;
//    pub1[7] = 0x0617031b;
}


void CRPT_IRQHandler(void)
{
    ECC_DriverISR(CRPT);
}

void EP2_Handler(void)  /* Interrupt IN handler */
{
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)((uint32_t)g_ISPInfo.rspbuf), EP2_MAX_PKT_SIZE);

    /* Trigger start Interrupt IN */
    USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);

}

void EP3_Handler(void)  /* Interrupt OUT handler */
{
    uint32_t    len;

    /* Interrupt OUT */
    if(g_ISPInfo.IsUSBDataReady == FALSE)
    {
        len = USBD_GET_PAYLOAD_LEN(EP3);

        USBD_MemCopy((uint8_t *)((uint32_t)&g_ISPInfo.rcvbuf), (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3)), len);

        g_ISPInfo.IsUSBDataReady = TRUE;

        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART1 interrupt event - To receive UART1 data                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    volatile uint32_t   u32UARTINTSTS, data_idx;

    data_idx = g_ISPInfo.UARTDataIdx;

    u32UARTINTSTS = UART1->INTSTS;

    if(u32UARTINTSTS & (UART_INTSTS_RDAIF_Msk | UART_INTSTS_RXTOIF_Msk))
    {
        //RDA FIFO interrupt & RDA timeout interrupt
        while(((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (data_idx < sizeof(CMD_PACKET_T))) //RX fifo not empty
        {
            g_ISPInfo.rcvbuf[data_idx++] = (uint8_t)UART1->DAT;
            g_ISPInfo.UARTDataIdx = data_idx;
        }
    }

    if(g_ISPInfo.UARTDataIdx == sizeof(CMD_PACKET_T))
    {
        g_ISPInfo.IsUARTDataReady = TRUE;
        g_ISPInfo.UARTDataIdx     = 0;
    }
    else if(u32UARTINTSTS & UART_INTSTS_RXTOIF_Msk)
    {
        g_ISPInfo.UARTDataIdx = 0;
    }
}

void SendUART1DataOut(uint8_t *pu8Buf)
{
    volatile uint32_t   i;

    for(i = 0; i < sizeof(CMD_PACKET_T); i++)
    {
        while((UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk)) {}
        UART1->DAT = pu8Buf[i];
    }
}

static void SendPacketToServer(ISP_INFO_T *pISPInfo)
{
    if(pISPInfo->IsUSBDataReady == TRUE)
    {
        EP2_Handler();
        pISPInfo->IsUSBDataReady = FALSE;
    }

    if(pISPInfo->IsUARTDataReady == TRUE)
    {
        SendUART1DataOut((uint8_t *)pISPInfo->rspbuf);
        pISPInfo->IsUARTDataReady = FALSE;
    }
}

static int32_t USBDISPInit(ISP_INFO_T *pISPInfo, uint32_t reserved)
{
    (void)reserved;
    printf("\n[*** Initial USBD ISP mode ***]\n");

    pISPInfo->IsUSBDataReady = FALSE;

    /* Use PLL as USB clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(2));

    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk | SYS_USBPHY_SBO_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_USB_VBUS | SYS_GPA_MFPH_PA13MFP_USB_D_N | SYS_GPA_MFPH_PA14MFP_USB_D_P | SYS_GPA_MFPH_PA15MFP_USB_OTG_ID);


    USBD_Open(&gsInfo, (CLASS_REQ)HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();

    NVIC_EnableIRQ(USBD_IRQn);

    USBD_Start();

    return 0;
}

static int32_t UART1ISPInit(ISP_INFO_T *pISPInfo)
{
    printf("\n[*** Initial UART1 ISP mode ***] (UART1 TX-PA.3; RX-PA.2)\n");

    /* UART1: TX = PA.3, RX = PA.2 */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk);
    SYS->GPA_MFPL |= (UART1_TXD_PA3 | UART1_RXD_PA2);

    /* Peripheral clock source, CLK_CLKSEL1_UART1SEL_HIRC: 12MHz */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL2_UART1SEL_Msk) | CLK_CLKSEL2_UART1SEL_HIRC;

    /* Enable peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk;


    pISPInfo->IsUARTDataReady = FALSE;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART function                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    UART1->FIFO = UART_FIFO_RFITL_14BYTES | UART_FIFO_RTSTRGLV_14BYTES;

    UART1->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(pISPInfo->UARTClockFreq, 115200));

    UART1->TOUT = 0x40;

    NVIC_SetPriority(UART1_IRQn, 2);
    NVIC_EnableIRQ(UART1_IRQn);

    UART1->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);

    return 0;
}




/* Stage 1. */
static int32_t ProcessConnect(ISP_INFO_T *pISPInfo)
{
    volatile int32_t    ret = 0, timeout;
    volatile ISP_INFO_T *pCurISPInfo;

    timeout = (int32_t)pISPInfo->timeout;

    pCurISPInfo = pISPInfo;

    pCurISPInfo->IsConnectOK = 0;

    printf("\n[In ProcessConnect]\n\n");
    while(1)
    {
        if((pCurISPInfo->IsUSBDataReady == TRUE) || (pCurISPInfo->IsUARTDataReady == TRUE))
        {
            if(pCurISPInfo->IsUSBDataReady == TRUE)
                printf("IS USB_CMD:\n\n");
            else
                printf("IS UART_CMD:\n\n");

            timeout = -1;
            ret = ParseCONNECT((ISP_INFO_T *)((uint32_t)pCurISPInfo));
            SendPacketToServer((ISP_INFO_T *)((uint32_t)pCurISPInfo));
        }

        if((ret == -1) || (ret == CMD_DISCONNECT) || (ret == CMD_CONNECT) || (ret == CMD_RESYNC))
            break;

        if(timeout != -1)
        {
            if(timeout-- == 0)
            {
                printf("[Connect time-out]\n\n");
                ret = -1;
                break;
            }
        }
    }

    return ret;
}

/* Stage 2. */
static int32_t ProcessECDH(ISP_INFO_T *pISPInfo)
{
    volatile int32_t    ret = 0;
    volatile ISP_INFO_T *pCurISPInfo;

    pCurISPInfo = pISPInfo;

    printf("\n[In ProcessECDH]\n\n");
    while(1)
    {
        if((pCurISPInfo->IsUSBDataReady == TRUE) || (pCurISPInfo->IsUARTDataReady == TRUE))
        {
            ret = ParseECDH((ISP_INFO_T *)((uint32_t)pCurISPInfo));
            SendPacketToServer((ISP_INFO_T *)((uint32_t)pCurISPInfo));
        }

        if((ret == -1) || (ret == CMD_DISCONNECT) || (ret == CMD_ECDH_GET_RAND_PUB1) || (ret == CMD_RESYNC))
            break;
    }

    return ret;
}

/* Stage 3. */
static int32_t ProcessCommands(ISP_INFO_T *pISPInfo)
{
    volatile int32_t    ret = 0;
    volatile ISP_INFO_T *pCurISPInfo;

    pCurISPInfo = pISPInfo;

    printf("\n[In ProcessCommands]\n\n");
    while(1)
    {
        if((pCurISPInfo->IsUSBDataReady == TRUE) || (pCurISPInfo->IsUARTDataReady == TRUE))
        {
            ret = ParseCommands((ISP_INFO_T *)((uint32_t)pCurISPInfo));
            SendPacketToServer((ISP_INFO_T *)((uint32_t)pCurISPInfo));
        }

        if((ret == -1) || (ret == CMD_DISCONNECT) || (ret == CMD_RESYNC))
            break;
    }

    return ret;
}

static int32_t ProcessISP(ISP_INFO_T *pISPInfo)
{
    volatile int32_t    ret = 0;

    CommandHandlerInit();

    do
    {
        /* Stage 1.*/
        ret = ProcessConnect(pISPInfo);
        if((ret == -1) || (ret == CMD_DISCONNECT) || (ret == CMD_RESYNC))
        {
            if(ret == -1)
                printf("\n[FAIL, ProcessConnect]\n");
            break;
        }

        /* Stage 2.*/
        ret = ProcessECDH(pISPInfo);
        if((ret == -1) || (ret == CMD_DISCONNECT) || (ret == CMD_RESYNC))
        {
            if(ret == -1)
                printf("\n[FAIL, ProcessECDH]\n");
            break;
        }

        /* Stage 3.*/
        ret = ProcessCommands(pISPInfo);
        if((ret == -1) || (ret == CMD_DISCONNECT) || (ret == CMD_RESYNC))
        {
            if(ret == -1)
                printf("\n[FAIL, ProcessCommands]\n");
            break;
        }
    }
    while(0);

    printf("\n[Exit ProcessISP]\n");

    return ret;
}

/**
  * @brief      Initial NuBL1 Secure ISP
  * @param      None
  * @retval     0           Success
  * @retval     -1          Failed
  * @details    This function initial the Secure ISP to update F/W.
  */
static int32_t SecureISPInit(ISP_INFO_T *pISPInfo, uint32_t reserved, E_ISP_MODE mode)
{
    volatile int32_t    ret = 0;

    (void)reserved;
    printf("\nIn [SecureISPInit mode: %d]\n", mode);

    /* Init USBD ISP mdoe ...... */
    if((mode & USB_MODE) == USB_MODE)
    {
        if((mode & RESYNC_ISP) == 0x0)
            USBDISPInit(pISPInfo, NULL);
    }

    /* Init UART1 ISP mdoe ...... */
    if((mode & UART_MODE) == UART_MODE)
    {
        if((mode & RESYNC_ISP) == 0x0)
            UART1ISPInit(pISPInfo);
    }

    ret = ProcessISP(pISPInfo);

    return ret;
}

int32_t ExecuteSecureISP(void)
{
    volatile int32_t    ret = 0;
    volatile uint32_t   ISPmode, u32UartClkFreq = 0;

    ISPmode = USB_UART_MODE;
    while(1)
    {
        if(ret == CMD_RESYNC)
        {
            memset((void *)((uint32_t)&g_ISPInfo.au32AESKey[0]), 0x0, sizeof(g_ISPInfo.au32AESKey));
            memset((void *)((uint32_t)&g_ISPInfo.au32AESIV[0]), 0x0, sizeof(g_ISPInfo.au32AESIV));
            memset((void *)((uint32_t)&g_ISPInfo.sign), 0x0, sizeof(ECDSA_SIGN_T));
            g_ISPInfo.UARTDataIdx = 0;
            g_ISPInfo.IsUSBDataReady = FALSE;
            g_ISPInfo.IsUARTDataReady = FALSE;
            g_ISPInfo.UARTClockFreq = u32UartClkFreq;

            ISPmode |= RESYNC_ISP;
        }
        else
        {
            memset((void *)((uint32_t)&g_ISPInfo), 0x0, sizeof(ISP_INFO_T));

            if((ISPmode & UART_MODE) == UART_MODE)
            {
                /* Configure UART1 clock source */
                g_ISPInfo.UARTClockFreq = __HIRC;
                u32UartClkFreq = g_ISPInfo.UARTClockFreq;
            }
        }

        /* Set Client's ECC public key */
        SetClientPublicKey((uint32_t *)((uint32_t)g_ISPInfo.ClientPubKey.au32Key0), (uint32_t *)((uint32_t)g_ISPInfo.ClientPubKey.au32Key1));

        /* Configure user's vendor function */
        g_ISPInfo.pfnVendorFunc = Exec_VendorFunction;

        g_ISPInfo.pfnGenKeyFunc = GenECDHSharedKey;

        /* Configure time-out time for checking the SecureISP Tool connection */
        g_ISPInfo.timeout = SystemCoreClock;

        ret = SecureISPInit((ISP_INFO_T *)((uint32_t)&g_ISPInfo), NULL, (E_ISP_MODE)ISPmode);
        if(ret == CMD_RESYNC)
            continue;

        if(ret < 0)
            return ret;

        break;
    }

    return 0;
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
