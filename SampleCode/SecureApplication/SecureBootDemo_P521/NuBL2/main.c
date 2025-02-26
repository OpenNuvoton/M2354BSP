/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to generate the first booting image, NuBL2 with ECC P-521 ECDSA signature.
 *           After NuBL2 runs via Secure Boot, NuBL2 will calculate CDI and compare it with the CDI value calculated by the Bootloader.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "NuBL2.h"

#define WRITE_ROTPK     0   /* Set 1 to program ROTPK */


static volatile FW_INFO_T  g_NuBL3xFwInfo; // Allocate a FWINFO buffer for storing NuBL32 FWINFO data

void SYS_Init(void);
void UART_Init(void);
void NuBL32_CDI(void);
extern int32_t CDI_Cal(uint8_t *salt, uint8_t *ikm, uint8_t *info, uint32_t *pbuf); // declared in cdi.c


/*
    ECC P-521 private key can refer to FwSign.ini:
        00df217363503e8135622a56e4c56cb8bfa850312a752bd22b48bef8f2a932af1b9f06a84b75f3c6dd8cf1e1620d4570fe5873b137a46ce973fe698df169cc2cf47d
*/
/* ECC P-521 public key pair stored in KS OTP 0~4 */
const uint32_t g_au32ROTPK[40] =
{
    // Public key 1 - 014e0eef3ad093dcb2fb9d28b73a6566c3089ade7d614992466663dfda01d731d3267506ad386b20a1160e965ba1d7f0b9072981a6c7ad1d85eb807e6c0225a9aa54
    0x25a9aa54, 0x807e6c02, 0xad1d85eb, 0x2981a6c7,
    0xd7f0b907, 0x0e965ba1, 0x6b20a116, 0x7506ad38,
    0xd731d326, 0x63dfda01, 0x49924666, 0x9ade7d61,
    0x6566c308, 0x9d28b73a, 0x93dcb2fb, 0x0eef3ad0, 0x0000014e,
    // Public key 2 - 00462e78c3295a5d9666ba4b1c689ad6d80fa71236f402468cf525b400cefa2e85349af7e83bbd6c3f9fda7033880a806664690c0e3a2c33f2aebb996908e2bee9bb
    0xe2bee9bb, 0xbb996908, 0x2c33f2ae, 0x690c0e3a,
    0x0a806664, 0xda703388, 0xbd6c3f9f, 0x9af7e83b,
    0xfa2e8534, 0x25b400ce, 0x02468cf5, 0xa71236f4,
    0x9ad6d80f, 0xba4b1c68, 0x5a5d9666, 0x2e78c329, 0x00000046,
    //Padding 6x32-bits
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x01ECC521 // Support CDI calculation
};

/* Example UDS for DICE */
const uint32_t g_au32UDS[] =
{
    0x25a9aa54, 0x807e6c02, 0xad1d85eb, 0x2981a6c7,
    0xd7f0b907, 0x0e965ba1, 0x6b20a116, 0x7506ad38
};




/**
  * @brief  Check if ROTPK present and write ROTPK
  */
static int32_t CheckROTPKStatus(void)
{
    /* Configure module clock */
    CLK_EnableModuleClock(KS_MODULE);

    KS_Open();

    if((KS->OTPSTS & 0x1F) == 0)
    {
        printf("ROTPK absent! NuBL2 execution via direct boot.\n\n");

        // TODO: User should set ROTPK in KS OTP0 ~ OTP4, and OTP7 used as UDS value for CDI calculation,
        //          so that the Secure Bootloader(NuBL1) can perform trusted boot to execute NuBL2.

        /*
            The NuBL2 ROTPK is a set of ECC P-521 public key pair.
            If the ECC P-521 private key is 00df217363503e8135622a56e4c56cb8bfa850312a752bd22b48bef8f2a932af1b9f06a84b75f3c6dd8cf1e1620d4570fe5873b137a46ce973fe698df169cc2cf47d, refer to FwSign.ini.
            The public key pair should be
                Public Key 1 = 014e0eef3ad093dcb2fb9d28b73a6566c3089ade7d614992466663dfda01d731d3267506ad386b20a1160e965ba1d7f0b9072981a6c7ad1d85eb807e6c0225a9aa54
                Public Key 2 = 00462e78c3295a5d9666ba4b1c689ad6d80fa71236f402468cf525b400cefa2e85349af7e83bbd6c3f9fda7033880a806664690c0e3a2c33f2aebb996908e2bee9bb

            Notes of programming NuBL2 ECC ROTPK:
            * The ROTPK arrary for KS_WriteOTP API
                const uint32_t g_au32ROTPK[40] =
                {
                    // Public key 1 - 014e0eef3ad093dcb2fb9d28b73a6566c3089ade7d614992466663dfda01d731d3267506ad386b20a1160e965ba1d7f0b9072981a6c7ad1d85eb807e6c0225a9aa54
                    0x25a9aa54, 0x807e6c02, 0xad1d85eb, 0x2981a6c7,
                    0xd7f0b907, 0x0e965ba1, 0x6b20a116, 0x7506ad38,
                    0xd731d326, 0x63dfda01, 0x49924666, 0x9ade7d61,
                    0x6566c308, 0x9d28b73a, 0x93dcb2fb, 0x0eef3ad0, 0x0000014e,
                    // Public key 2 - 00462e78c3295a5d9666ba4b1c689ad6d80fa71236f402468cf525b400cefa2e85349af7e83bbd6c3f9fda7033880a806664690c0e3a2c33f2aebb996908e2bee9bb
                    0xe2bee9bb, 0xbb996908, 0x2c33f2ae, 0x690c0e3a,
                    0x0a806664, 0xda703388, 0xbd6c3f9f, 0x9af7e83b,
                    0xfa2e8534, 0x25b400ce, 0x02468cf5, 0xa71236f4,
                    0x9ad6d80f, 0xba4b1c68, 0x5a5d9666, 0x2e78c329, 0x00000046,
                    //Padding 6x32-bits
                    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x01ECC521 // Support CDI calculation
                };

            * An example to program KS OPT0 ~ OPT1, and OTP7
                // Configure module clock
                CLK_EnableModuleClock(KS_MODULE);
                KS_Open();
                KS_WriteOTP(0, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[0]);
                KS_WriteOTP(1, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[8]);
                KS_WriteOTP(2, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[16]);
                KS_WriteOTP(3, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[24]);
                KS_WriteOTP(4, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[32]);
                // UDS for CDI calculation
                KS_WriteOTP(7, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32UDS[0]);
        */
#if (WRITE_ROTPK == 1)
        printf("Hit [K] to program ROTPK.\n\n");
        if(getchar() != 'K')
        {
            return 0;
        }

        // Configure module clock
        CLK_EnableModuleClock(KS_MODULE);

        KS_Open();
        KS_WriteOTP(0, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[0]);
        KS_WriteOTP(1, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[8]);
        KS_WriteOTP(2, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[16]);
        KS_WriteOTP(3, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[24]);
        KS_WriteOTP(4, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[32]);
        // UDS for CDI calculation.
        KS_WriteOTP(7, (KS_META_256 | KS_META_CPU | KS_META_READABLE | KS_META_BOOT), (uint32_t *)&g_au32UDS[0]);

        printf("Program ROTPK ... Done.\n\n");
#endif

        return 0;
    }
    else
    {
        printf("ROTPK present. NuBL2 execution via secure boot.\n\n");

        return 1;
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

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open((UART_T *)DEBUG_PORT, 115200);
}

/**
  * @brief  Calculate a SHA-512 message digest
  */
static int32_t SHA512(uint32_t addr, uint32_t size, uint8_t digest[])
{
    volatile int32_t    i, timeout = (100000 * 5);
    uint32_t            Hash[16];

    // Sync output byte order with PC
    CRPT->HMAC_CTL    = ((SHA_MODE_SHA512 << CRPT_HMAC_CTL_OPMODE_Pos) | CRPT_HMAC_CTL_INSWAP_Msk | CRPT_HMAC_CTL_OUTSWAP_Msk);
    CRPT->HMAC_SADDR  = (addr);
    CRPT->HMAC_DMACNT = (size);

    CRPT->HMAC_CTL &= ~(0x7UL << CRPT_HMAC_CTL_DMALAST_Pos);
    CRPT->HMAC_CTL |= (CRPT_HMAC_CTL_START_Msk | (0xBUL << CRPT_HMAC_CTL_DMAFIRST_Pos));
    while((CRPT->HMAC_STS & CRPT_HMAC_STS_BUSY_Msk) == CRPT_HMAC_STS_BUSY_Msk)
    {
        if(timeout-- < 1)
        {
            return -1;
        }
    }

    for(i = 0; i < 16; i++)
        Hash[i] = *(uint32_t *)((uint32_t) & (CRPT->HMAC_DGST[0]) + (i * 4));

    (void)memcpy(digest, (uint32_t *)((uint32_t)Hash), 64);

    return 0;
}

//#define cdidbg      printf
#define cdidbg(...)

/**
  * @brief  Calculate a CDI value
  */
void NuBL32_CDI(void)
{
    volatile uint32_t i;
    uint32_t cdi_cal[8], ks_data[8];
    uint8_t *pu8;
    uint8_t uds[32];        // KS OTP-7
    uint8_t hiddeninfo[15]; // First 15 bytes in KS OTP-0
    uint32_t cfg[4];
    /* "NuvotonM2354BL32" */
    uint8_t info[16] = {0x4e, 0x75, 0x76, 0x6f, 0x74, 0x6f, 0x6e, 0x4d, 0x32, 0x33, 0x35, 0x34, 0x42, 0x4c, 0x33, 0x32};
    uint32_t salt_src[(96 / 4)];
    uint32_t hmac[32];
    uint8_t salt[64];


    /* Calculate CDI value by NuBL2 */

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    FMC_Open();

    KS_Read(KS_OTP, 7, ks_data, 8);
    memcpy(uds, &ks_data[0], 32);
    cdidbg("uds: ");
    for(i = 0; i < 32; i++)
        cdidbg("%02x", uds[i]);
    cdidbg("\n================================================================\n");

    memcpy(hiddeninfo, &g_au32ROTPK[0], 15);
    cdidbg("hiddeninfo: ");
    for(i = 0; i < 15; i++)
        cdidbg("%02x", hiddeninfo[i]);
    cdidbg("\n================================================================\n");

    pu8 = (uint8_t *)(info);
    cdidbg("info: ");
    for(i = 0; i < 16; i++)
        cdidbg("%c", pu8[i]);
    cdidbg("\n================================================================\n");

    /*
         Input (salt) (96 bytes)
            SHA512 Hash of Layer 0 (64 bytes)
            User config0, 1, 2, 3 (16 bytes)
            Mode byte (1 byte) ==> 0x01
                0: Not configured, 1:Normal, 2:Debug, 3:Recovery
            Hidden Inputs (15 bytes) ==> 15 bytes of ROTPK
                User defined

        Salt = SHA512(Input)
    */
    pu8 = (uint8_t *)salt_src;

    // Copy "NuBL32 FW SHA-512 from FW Info"
    memcpy(pu8, (void*)&g_NuBL3xFwInfo.au32FwHash[0], 64);
    // Copy "User CFG"
    cfg[0] = FMC_Read(FMC_USER_CONFIG_0);
    cfg[1] = FMC_Read(FMC_USER_CONFIG_1);
    cfg[2] = FMC_Read(FMC_SCRLOCK_BASE);
    cfg[3] = FMC_Read(FMC_ARLOCK_BASE);
    memcpy((void *)(pu8 + 64 + 0), (void *)&cfg[0], 4);
    memcpy((void *)(pu8 + 64 + 4), (void *)&cfg[1], 4);
    memcpy((void *)(pu8 + 64 + 8), (void *)&cfg[2], 4);
    memcpy((void *)(pu8 + 64 + 12), (void *)&cfg[3], 4);
    // Copy "Mode byte"
    pu8[64 + 16] = 0x1;
    // Copy "Hidden Inputs"
    memcpy((void *)(pu8 + 64 + 16 + 1), (void *)&hiddeninfo[0], 15);

    cdidbg("salt_src:\n");
    for(i = 0; i < 24; i++)
        cdidbg("%2d: 0x%08x\n", i, salt_src[i]);
    cdidbg("================================================================\n");

    /* Calculate SHA-512 */
    SHA512((uint32_t)(salt_src), 96, salt);

    /* Calculate a CDI value */
    CDI_Cal(salt, uds, info, &hmac[0]);
    memcpy(cdi_cal, hmac, 32);

    printf("\nThe NuBL32 CDI calculated by NuBL2: \n");
    printf("================================================================\n");
    for(i = 0; i < 8; i++)
        printf("CDI-%d: 0x%08x\n", i, cdi_cal[i]);
    printf("================================================================\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32NuBL32Base, u32TimeOutCnt;
    volatile uint32_t i;
    uint32_t cdi_get[8], cdi_cal[8], ks_data[8];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %d Hz (Non-secure flash base: 0x%x)\n", SystemCoreClock, SCU->FNSADDR);
    printf("+---------------------------------------------------------------------------+\n");
    printf("|    SecureBootDemo - NuBL2 Sample Code, P-521 ECDSA and CDI calculation    |\n");
    printf("+---------------------------------------------------------------------------+\n\n");

    if(CheckROTPKStatus() == 1)
    {

        /* Secure boot into NuBL2, get NuBL2 CDI value calculated by the Bootloader */

        /* Enable KS module clock */
        CLK_EnableModuleClock(KS_MODULE);

        KS_Open();
        KS_Read(KS_SRAM, 0, cdi_get, 8);

        printf("\nThe NuBL2 CDI calculated by Secure Bootloader: \n");
        printf("================================================================\n");
        for(i = 0; i < 8; i++)
            printf("CDI-%d: 0x%08x\n", i, cdi_get[i]);
        printf("================================================================\n");
    }

    /* Authenticate NuBL32 FW */
    memcpy((void *)((uint32_t)&g_NuBL3xFwInfo), (void *)NUBL32_FW_INFO_BASE, sizeof(FW_INFO_T));
    if(VerifyNuBL3x((uint32_t *)((uint32_t)&g_NuBL3xFwInfo), NUBL32_FW_INFO_BASE) == -1)
    {
        printf("\n\nNuBL2 verifies NuBL32 FAIL.\n");
        goto lexit;
    }
    else
    {
        u32NuBL32Base = g_NuBL3xFwInfo.mData.au32FwRegion[0].u32Start;
        printf("\nNuBL2 verifies NuBL32 FW PASS.\n");

        /* Calculate NuBL32 CDI value by the NuBL2 */
        NuBL32_CDI();
    }

    /* Jump to execute NuBL32 FW */
    printf("\nJump to execute NuBL32...\n");
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY((UART_T *)DEBUG_PORT)
    if(--u32TimeOutCnt == 0) break;

    /* Disable all interrupt */
    __set_PRIMASK(1);

    FMC_ENABLE_ISP();
    FMC_SetVectorPageAddr(u32NuBL32Base);

    /* Reset to execute NuBL32 FW */
    NVIC_SystemReset();

lexit:

    while(1) {}
}
