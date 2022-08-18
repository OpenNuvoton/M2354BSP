/**************************************************************************//**
 * @file     NuBL2_main.c
 * @version  V1.00
 * @brief    Demonstrate to do secure OTA update for NuBL32 and NuBL33 firmware by NuBL2 with bank remap.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <arm_cmse.h>
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "partition_M2354.h"

#include "NuBL_common.h"
#include "NuBL_crypto.h"
#include "NuBL2.h"

#include "ota.h"

volatile uint32_t gNuBL2_32Key[8], gNuBL2_33Key[8];

static volatile ISP_INFO_T g_NuBL2ISPInfo = {0};

#define PLL_CLOCK       64000000

/* For ECC NIST: Curve P-256 */
const uint32_t g_au32Eorder[] =
{
    0xFC632551, 0xF3B9CAC2, 0xA7179E84, 0xBCE6FAAD, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF
};

void CRPT_IRQHandler(void);
void WDT_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);
void HardFault_Handler(void)__attribute__((noreturn));
void WdtEnableAndCheck(void);
int8_t NuBLTrustBootInit(void);
__NONSECURE_ENTRY
void BL2_ECC_ISR(void);
int8_t FWCopy(uint32_t u32SrcAddr, uint32_t u32DstAddr, uint32_t u32Size);
int8_t FlashBankRemapCheck(void);
__NONSECURE_ENTRY
int32_t GetBLxFwVer(uint32_t * pu32FwVer, uint8_t u8Mode);


void CRPT_IRQHandler(void)
{
    printf("CRPT_IRQHandler in NuBL2\n");
    ECC_DriverISR(CRPT);
}

__NONSECURE_ENTRY
void BL2_ECC_ISR(void)
{
    ECC_DriverISR(CRPT);
}


/**
 * @brief       IRQ Handler for WDT Interrupt
 * @param       None
 * @return      None
 * @details     The WDT_IRQHandler is default IRQ of WDT, declared in startup_M2354.s.
 */
void WDT_IRQHandler(void)
{
    printf("WDT_IRQHandler\n");
#if (WDT_RST_ENABLE)
    if(g_u32WakeupCounts < 10)
    {
        WDT_RESET_COUNTER();
    }

    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();

        g_u8IsINTEvent = 1;
    }

    if(WDT_GET_TIMEOUT_WAKEUP_FLAG() == 1)
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();

        g_u32WakeupCounts++;
    }
#endif
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
    #if defined( NUMAKER_BOARD )
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;
    #elif defined( NUMAKER_IOT_BOARD )
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB8_Msk | UART0_TXD_PB9_Msk))) | UART0_RXD_PB8 | UART0_TXD_PB9;
    #else
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;
    #endif
}

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

void HardFault_Handler(void)
{
    printf("NuBL2 HardFault!!\n");
    while(1) {}
}

/**
  * @brief      Enable WDT and check if reset by WDT
  * @param      None
  * @return     None
  * @details    Enable WDT and check if system reset by WDT
  */
void WdtEnableAndCheck(void)
{
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
#if (WDT_RST_ENABLE)
    /* Check WDT config */
    u32Cfg = FMC_Read(FMC_CONFIG_BASE);
    if((u32Cfg & (BIT31 | BIT4 | BIT3)) == ((BIT31 | BIT4 | BIT3)))
    {
        printf("[WRAN]WDT was disabled\n");
        FMC_ENABLE_CFG_UPDATE();
        u32Cfg &= ~((BIT31 | BIT4 | BIT3));
        FMC_Write(FMC_CONFIG_BASE, u32Cfg);

        //NVIC_SystemReset();
        SYS_ResetChip();
        while(1) {}
    }
    else
        printf("WDT was enabled\n");
#endif
    if(SYS_IS_WDT_RST())
    {
        /* upgrade done, clear OTA status */
        FMC_Write(OTA_STATUS_BASE, 0x1);

        SYS_ClearResetSrc(SYS_RSTSTS_WDTRF_Msk);
        printf("*** System has been reset by WDT time-out event 0***\n\n");
        SYS_ResetChip();
        while(1) {}
    }

    /* To check if system has been reset by WDT time-out reset or not */
    if(WDT_GET_RESET_FLAG() == 1)
    {
        WDT_CLEAR_RESET_FLAG();

        /* upgrade done, clear OTA status */
        FMC_Write(OTA_STATUS_BASE, 0x1);

        printf("*** System has been reset by WDT time-out event ***\n\n");
        SYS_ResetChip();
        while(1) {}
    }
}

/**
  * @brief      Trust boot initialization
  * @param      None
  * @retval     0           Success
  * @retval     Other       Fail
  * @details    Create FW INFO
  */
//FwSign.exe .\NuBL2_FW.bin\FLASH .\NuBL2_FW.bin\FWINFO
int8_t NuBLTrustBootInit(void)
{
    /* Init NuBL2 FW INFO */
    extern const uint32_t g_InitialFWinfo[];
    //uint32_t cfg = g_InitialFWinfo.mData.u32AuthCFGs;
    uint32_t cfg = g_InitialFWinfo[16];
    printf("\n[AuthCFG: 0x%08x]\n", cfg);

    return 0;
}

/**
  * @brief      Firmware copy
  * @param      None
  * @retval     0           Success
  * @retval     Other       Fail
  * @details    Copy NuBL32/NuBL33 firmware
  */
int8_t FWCopy(uint32_t u32SrcAddr, uint32_t u32DstAddr, uint32_t u32Size)
{
    uint32_t    u32ReadAddr, u32WriteAddr;
    uint32_t    u32Data, u32CopyData;

    SYS_UnlockReg();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* Erase firmware */
    u32WriteAddr = u32DstAddr;
    for(u32WriteAddr = u32DstAddr; u32WriteAddr < (u32DstAddr + u32Size); u32WriteAddr += FMC_FLASH_PAGE_SIZE)
    {
        FMC_Erase(u32WriteAddr);
    }

    /* Copy firmware */
    u32WriteAddr = u32DstAddr;
    for(u32ReadAddr = u32SrcAddr; u32ReadAddr < (u32SrcAddr + u32Size); u32ReadAddr += 4, u32WriteAddr += 4)
    {
        u32Data = FMC_Read(u32ReadAddr);
        FMC_Write(u32WriteAddr, u32Data);
    }

    /* verify copied firmware */
    u32WriteAddr = u32DstAddr;
    for(u32ReadAddr = u32SrcAddr; u32ReadAddr < (u32SrcAddr + u32Size); u32ReadAddr += 4, u32WriteAddr += 4)
    {
        u32Data = FMC_Read(u32ReadAddr);
        u32CopyData = FMC_Read(u32WriteAddr);
        if(u32Data != u32CopyData)
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32WriteAddr, u32CopyData, u32Data);
            return -1;
        }
    }

    return 0;
}


/**
  * @brief      Get NuBL32 or NuBL33 F/W Version
  * @param[in]  * pu32FwVer F/W version write buffer  \n
  * @param[in]  u8Mode      F/W version of NuBL32 or NuBL33. bit-0: 0: NuBL32; 1: NuBL33  \n
  *                         bit-2: 0: Bank0; 1: Bank1
  * @retval     0           Success
  * @retval     others      Failed
  * @details    This function is used to get F/W version of NuBL32 or NuBL33. \n
  *             Flow: \n
  *                 1. Get NuBL3x info \n
  *                 2. Get NuBL3x F/W version (enclosed in F/W info) \n
  */
__NONSECURE_ENTRY
int32_t GetBLxFwVer(uint32_t * pu32FwVer, uint8_t u8Mode)
{
    volatile int32_t    i, ret = -1000;
    uint32_t            *infobuf, base, len;
    FW_INFO_T           FwInfo;

    if(!((u8Mode == 0) || (u8Mode == 1) || (u8Mode == BIT2) || (u8Mode == (1 | BIT2))))
    {
        NUBL_MSG("\nGet NuBL3x Version FAIL. Invalid mode: 0x%x.\n\n", u8Mode);
        return ret;
    }

    NUBL_MSG("\nGet NuBL3%d. \n\n", ((u8Mode & BIT0) == 0) ? 2 : 3);

    memset(&FwInfo, 0x0, sizeof(FW_INFO_T));

    infobuf = (uint32_t *)(uint32_t)&FwInfo;

    /* Step 1. Get NuBL3x info */
    /* Get NuBL3x F/W info */
    len = sizeof(FW_INFO_T);
    if((u8Mode & BIT4) != BIT4)
    {
        FMC_Open();
        if((u8Mode & BIT0) == 0)
            if((u8Mode & BIT2) == 0)
                base = NUBL32_FW_INFO_BASE;   // encrypted NuBL32 info address
            else
                base = NUBL32_FW_INFO_BASE | FMC_BANK_SIZE;   // encrypted NuBL32 info address
        else if((u8Mode & BIT2) == 0)
            base = NUBL33_FW_INFO_BASE;   // encrypted NuBL33 info address
        else
            base = NUBL33_FW_INFO_BASE | FMC_BANK_SIZE;   // encrypted NuBL33 info address

        for(i = 0; i < ((int32_t)len / 4); i++)
            infobuf[i] = FMC_Read(base + ((uint32_t)i * 4));
    }
    NUBL_MSG("Get NuBL3%d F/W info [Done]\n\n", ((u8Mode & BIT0) == 0) ? 2 : 3);

    /* Step 2. Get NuBL3x F/W version (enclosed in F/W info) */
    memcpy(&FwInfo, infobuf, sizeof(FW_INFO_T));
    memcpy(pu32FwVer, &FwInfo.mData.au32ExtInfo[0], sizeof(uint32_t));

    NUBL_MSG(" NuBL3%d F/W version is [0x%08x]\n\n", ((u8Mode & BIT0) == 0) ? 2 : 3, FwInfo.mData.au32ExtInfo[0]);

    ret = 0;

    return ret;
}


/**
  * @brief      Check and to do bank remap or not
  * @param      None
  * @retval     0       don't need bank remap
  * @retval     1       do bank remap
  * @details    Check NuBL32/NuBL33 then decide to do bank remap or not.
  */
int8_t FlashBankRemapCheck(void)
{
    uint32_t u32NuBL32Bank0Pass, u32NuBL33Bank0Pass, u32NuBL32Bank1Pass, u32NuBL33Bank1Pass;
    uint32_t u32NuBL32Bank0Ver, u32NuBL33Bank0Ver, u32NuBL32Bank1Ver, u32NuBL33Bank1Ver;

    u32NuBL32Bank0Pass = u32NuBL33Bank0Pass = u32NuBL32Bank1Pass = u32NuBL33Bank1Pass = 1;

    /* verify NuBL32 in Flash bank0 */
    if(NuBL2_ExecuteVerifyNuBL3xBankx(NULL, 0, 0) != 0)
        u32NuBL32Bank0Pass = 0;

    /* verify NuBL33 in Flash bank0 */
    if(NuBL2_ExecuteVerifyNuBL3xBankx(NULL, 1, 0) != 0)
        u32NuBL33Bank0Pass = 0;

    /* verify NuBL32 in Flash bank1 */
    if(NuBL2_ExecuteVerifyNuBL3xBankx(NULL, 0, 1) != 0)
        u32NuBL32Bank1Pass = 0;

    /* verify NuBL33 in Flash bank1 */
    if(NuBL2_ExecuteVerifyNuBL3xBankx(NULL, 1, 1) != 0)
        u32NuBL33Bank1Pass = 0;

    printf("[Bank0]\n\tNuBL32 verify Pass: %d, NuBL33 verify Pass: %d\n[Bank1]\n\tNuBL32 verify Pass: %d, NuBL33 verify Pass: %d\n\n", \
           u32NuBL32Bank0Pass, u32NuBL33Bank0Pass, u32NuBL32Bank1Pass, u32NuBL33Bank1Pass);
#if 0
    if(u32NuBL32Bank0Pass > u32NuBL32Bank1Pass)
    {
        /* NuBL32 FWINFO */
        if(FWCopy(NUBL32_FW_INFO_BASE, NUBL32_FW_INFO_BASE | FMC_BANK_SIZE, FW_INFO_SIZE) != 0)
        {
            printf("Copy NuBL32 FWINFO to bank1 has error!\n");
            return 0;
        }
        if(FWCopy(NUBL32_FW_BASE, NUBL32_FW_BASE | FMC_BANK_SIZE, SYS_NEW_FW_BLOCK_SIZE) != 0)
        {
            printf("Copy NuBL32 to bank1 has error!\n");
            return 0;
        }
        printf("Copy NuBL32 and FWINFO to bank1\n");
    }

    if(u32NuBL33Bank0Pass > u32NuBL33Bank1Pass)
    {
        /* NuBL33 FWINFO */
        if(FWCopy(NUBL33_FW_INFO_BASE, NUBL33_FW_INFO_BASE | FMC_BANK_SIZE, FW_INFO_SIZE) != 0)
        {
            printf("Copy NuBL33 FWINFO to bank1 has error!\n");
            return 0;
        }
        if(FWCopy(NUBL33_FW_BASE, NUBL33_FW_BASE | FMC_BANK_SIZE, APP_NEW_FW_BLOCK_SIZE) != 0)
        {
            printf("Copy NuBL33 to bank1 has error!\n");
            return 0;
        }
        printf("Copy NuBL33 and FWINFO to bank1\n");
    }

    if(u32NuBL32Bank1Pass > u32NuBL32Bank0Pass)
    {
        /* NuBL32 FWINFO */
        if(FWCopy(NUBL32_FW_INFO_BASE | FMC_BANK_SIZE, NUBL32_FW_INFO_BASE, FW_INFO_SIZE) != 0)
        {
            printf("Copy NuBL32 FWINFO to bank0 has error!\n");
            return 0;
        }
        if(FWCopy(NUBL32_FW_BASE | FMC_BANK_SIZE, NUBL32_FW_BASE, SYS_NEW_FW_BLOCK_SIZE) != 0)
        {
            printf("Copy NuBL32 to bank0 has error!\n");
            return 0;
        }
        printf("Copy NuBL32 and FWINFO to bank0\n");
    }

    if(u32NuBL33Bank1Pass > u32NuBL33Bank0Pass)
    {
        /* NuBL33 FWINFO */
        if(FWCopy(NUBL33_FW_INFO_BASE | FMC_BANK_SIZE, NUBL33_FW_INFO_BASE, FW_INFO_SIZE) != 0)
        {
            printf("Copy NuBL33 FWINFO to bank0 has error!\n");
            return 0;
        }
        if(FWCopy(NUBL33_FW_BASE | FMC_BANK_SIZE, NUBL33_FW_BASE, APP_NEW_FW_BLOCK_SIZE) != 0)
        {
            printf("Copy NuBL33 to bank0 has error!\n");
            return 0;
        }
        printf("Copy NuBL33 and FWINFO to bank0\n");
    }
#endif
    /* Compare Firmware version */
    if(GetBLxFwVer(&u32NuBL32Bank0Ver, 0) != 0)
    {
        printf("Get NuBL32 version in bank0 has error!\n");
        goto _FAIL;
    }

    if(GetBLxFwVer(&u32NuBL32Bank1Ver, 0 | BIT2) != 0)
    {
        printf("Get NuBL32 version in bank1 has error!\n");
        goto _FAIL;
    }

    if(GetBLxFwVer(&u32NuBL33Bank0Ver, 1) != 0)
    {
        printf("Get NuBL33 version in bank0 has error!\n");
        goto _FAIL;
    }

    if(GetBLxFwVer(&u32NuBL33Bank1Ver, 1 | BIT2) != 0)
    {
        printf("Get NuBL33 version in bank1 has error!\n");
        goto _FAIL;
    }

    printf("[Bank0]\n\tNuBL32 Version: 0x%08x, NuBL33 Version: 0x%08x\n", u32NuBL32Bank0Ver, u32NuBL33Bank0Ver);
    printf("[Bank1]\n\tNuBL32 Version: 0x%08x, NuBL33 Version: 0x%08x\n\n", u32NuBL32Bank1Ver, u32NuBL33Bank1Ver);

    /* The BL32 firmware version in bank1 is newer and the newer firmware is verified pass. */
    if((u32NuBL32Bank1Ver > u32NuBL32Bank0Ver) && (u32NuBL32Bank1Ver != 0xFFFFFFFF) && \
        (u32NuBL32Bank1Pass != 0) && (u32NuBL33Bank1Pass != 0))
        return 1; /* Do bank remap */

    /* The BL33 firmware version in bank1 is newer and the newer firmware is verified pass. */
    if((u32NuBL33Bank1Ver > u32NuBL33Bank0Ver) && (u32NuBL33Bank1Ver != 0xFFFFFFFF) && \
        (u32NuBL33Bank1Pass != 0) && (u32NuBL32Bank1Pass != 0))
        return 1; /* Do bank remap */

    /* The BL32 or BL33 firmware in bank0 is verified failed. */
    if ((u32NuBL32Bank0Pass == 0) || (u32NuBL33Bank0Pass == 0))
    {
        /* Check if the BL32 and BL33 firmware in bank1 are verified pass. */
        if ((u32NuBL32Bank1Pass != 0) && (u32NuBL33Bank1Pass != 0))
            return 1; /* Do bank remap */
    }

    return 0; /* Don't bank remap */
_FAIL:
    printf("FlashBankRemapCheck fail!\n");
    return 0; /* Don't bank remap */
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32NeedReset = 0, u32TimeOutCnt;
    uint8_t u8FailNuBL3x; /* Bit0 = 1: NuBL32, Bit1 = 1: NuBL33 */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n[HCLK %d Hz] \n", SystemCoreClock);
    printf("+------------------------------------+\n");
    printf("|    M2354 Secure OTA Sample Code    |\n");
    printf("+------------------------------------+\n\n");
    printf("* Compile define for WiFi module pin selection:\n");
    printf(" \tNUMAKER_BOARD     : for NuMaker board.\n");
    printf(" \tNUMAKER_IOT_BOARD : for NuMaker-IOT board.\n\n");
    #if defined( NUMAKER_BOARD )
    printf("* Current WiFi module pin selection is for NuMaker board.\n\n");
    #elif defined( NUMAKER_IOT_BOARD )
    printf("* Current WiFi module pin selection is for NuMaker-IOT board.\n\n");
    #else
    printf("* Current WiFi module pin selection is for NuMaker board.\n\n");
    #endif

    CLK_SysTickDelay(200000);

    /* Create FW INFO */
    NuBLTrustBootInit();

    SYS_UnlockReg();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* Enable Flash mirror boundary */
    if((FMC->ISPSTS & FMC_ISPSTS_MIRBOUND_Msk) == 0)
    {
        printf("Please enable Flash mirror boundary by ICP programming tool first.(FMC ISPSTS:0x%08x)\n", FMC->ISPSTS);
        while(1);
    }

    /* Enable WDT and check if system reset by WDT */
    WdtEnableAndCheck();

    /* Enable CRYPTO power */
    SYS->PSWCTL |= SYS_PSWCTL_CRPTPWREN_Msk;

    /* Enable CRYPTO clock */
    CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;

    ECC_ENABLE_INT(CRPT);

    NVIC_EnableIRQ(CRPT_IRQn);

    printf("check OTA status:%d\n", FMC_Read(OTA_STATUS_BASE));

    /* check NuBL32/NuBL33 then decide to do bank remap or not. */
    if(FlashBankRemapCheck() == 1)
    {
        FMC_RemapBank(0x1);
    }

    printf(" ------ Code on bank %d ------\n", (FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) ? 1 : 0);

    /* normal boot */
    /* verify application firmware identity and integrity */
    NuBL2_Init();
    /* verify system firmware identity and integrity */
    if(NuBL2_ExecuteVerifyNuBL3x(NULL, 0) != 0)
    {
        /* if verify failed, do OTA update right now */
        printf("\n\nNuBL2 verifies NuBL32 FAIL.\n\n");
        /* set this flag for reset NuBL32 firmware version to 0. */
        u8FailNuBL3x = BIT0;

        goto _VERIFY_FAIL;
    }
    else
    {
        printf("\n\nNuBL2 verifies NuBL32 PASS.\n\n");
    }

    if(NuBL2_ExecuteVerifyNuBL3x(NULL, 1) != 0)
    {
        /* if verify failed, do OTA update right now */
        printf("\n\nNuBL2 verifies NuBL33 FAIL.\n\n");
        /* set this flag for reset NuBL33 firmware version to 0. */
        u8FailNuBL3x = BIT1;

        goto _VERIFY_FAIL;
    }
    else
    {
        printf("\n\nNuBL2 verifies NuBL33 PASS.\n\n");
    }

    printf("\n[Executing in NuBL2] execute NuBL32 F/W on 0x%08x.\n", (uint32_t)NUBL32_FW_BASE);

    __set_PRIMASK(1); /* Disable all interrupt */
    FMC_SetVectorPageAddr(NUBL32_FW_BASE);

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();
    while(1) {}

_VERIFY_FAIL:
#if 1
    {
        /* debug */
        while(1) {}
    }
#else
    /* if verify failed, modified NuBL3x firmware version to 0, and do OTA update after reboot */
    if(u8FailNuBL3x & BIT0)
    {
        FW_INFO_T FwInfo;

        /* Clear NuBL3x firmware version to 0 */
        FwInfo.mData.au32ExtInfo[0] = 0;
        /* NuBL32 */
        if(NuBL2_UpdateNuBL3xFwInfo((uint32_t *)&FwInfo, sizeof(FW_INFO_T), 0, NUBL32_FW_INFO_BASE) != 0)
        {
            printf("\nClear NuBL32 F/W version failed.\n");
        }
    }

    if((u8FailNuBL3x & BIT1))
    {
        FW_INFO_T FwInfo;

        /* Clear NuBL3x firmware version to 0 */
        FwInfo.mData.au32ExtInfo[0] = 0;
        /* NuBL33 */
        if(NuBL2_UpdateNuBL3xFwInfo((uint32_t *)&FwInfo, sizeof(FW_INFO_T), 1, NUBL33_FW_INFO_BASE) != 0)
        {
            printf("\nClear NuBL33 F/W version failed.\n");
        }
    }

    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* update OTA status for upgrade right now. */
    FMC_Write(OTA_STATUS_BASE, 0x1UL);

    /* init OTA */
    OTA_Init(__HSI, (ISP_INFO_T *)&g_NuBL2ISPInfo);
    if(OTA_TaskProcess() == 0)
    {
        /* check OTA status and re-boot for update firmware */
        printf("OTA update for NuBL3x verify failed: SYS:%d, APP:%d\n", FMC_Read(SYS_FW_OTA_STATUS_BASE), FMC_Read(APP_FW_OTA_STATUS_BASE));
        if((FMC_Read(SYS_FW_OTA_STATUS_BASE) == 1) || FMC_Read(APP_FW_OTA_STATUS_BASE) == 1)
        {
            u32NeedReset = TRUE;
        }
    }
    goto reset;

    while(1) {}
#endif
reset:
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(UART_IS_TX_EMPTY(DEBUG_PORT)))
        if( --u32TimeOutCnt == 0 ) break;

    /* Reset CPU only to reset to new vector page */
    SYS_ResetChip();

    while(1) {}
}

////////////////////////////////////////////////////////////////////////////////////////////

/**
  * @brief      Initial NuBL2
  * @param      None
  * @retval     0           Success
  * @retval     -1          Failed
  */
int32_t NuBL2_Init(void)
{
    uint32_t au32CFG[4], u32XOMStatus;

    if((sizeof(METADATA_T) % 16) != 0)
        return -1;

    if((sizeof(FW_INFO_T) % 16) != 0)
        return -2;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    FMC_ReadConfig(au32CFG, 4);

    printf("Chip info:\n");
    printf("\t[Secure boundary : 0x%08x]\n", SCU->FNSADDR);
    printf("\t[CONFIG0         : 0x%08x]", au32CFG[0]);
    if(!(au32CFG[0]&BIT5))
    {
        printf("\tBoot from MaskROM -> %s.\n", (au32CFG[0]&BIT7) ? "APROM" : "LDROM");
    }
    else
    {
        if((au32CFG[0]&BIT7))
            printf("\tBoot from APROM.\n");
        else
            printf("\tBoot from LDROM.\n");
    }

    u32XOMStatus = FMC_GetXOMState(XOMR0);
    printf("\t[XOM0            : %s]", (u32XOMStatus == 1) ? "Active" : "Inactive");
    printf("\tBase: 0x%08x; Page count: 0x%08x.\n", (FMC->XOMR0STS >> 8), (FMC->XOMR0STS & 0xFF));
    u32XOMStatus = FMC_GetXOMState(XOMR1);
    printf("\t[XOM1            : %s]", (u32XOMStatus == 1) ? "Active" : "Inactive");
    printf("\tBase: 0x%08x; Page count: 0x%08x.\n", (FMC->XOMR1STS >> 8), (FMC->XOMR1STS & 0xFF));

    return 0;
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
