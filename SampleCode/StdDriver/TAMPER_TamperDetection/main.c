/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of TAMPER static tamper interrupt to wake up system
 *           or clear keys in SRAM of Key Store.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
static volatile uint8_t s_u8Option;

int32_t KS_TRIG(void);
void TAMPER_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);
int32_t KS_Init(void);
int32_t KeyStoreSRAM(void);

int32_t KS_TRIG(void)
{
    uint32_t u32TimeOutCnt;

    /* Check BUSY(KS_STS[2]) is 0 and EIF(KS_STS[1]) is 0 */
    if(KS->STS & (KS_STS_BUSY_Msk | KS_STS_EIF_Msk))
    {
        printf("Key store status is wrong!\n");
    }

    /* Set OPMODE(KS_CTL[3:1]) to 000 */
    KS->CTL = (KS->CTL & (~KS_CTL_OPMODE_Msk)) | (0 << KS_CTL_OPMODE_Pos);

    /* Set START(KS_CTL[0]) to 1 */
    KS->CTL |= (1 << KS_CTL_START_Pos);

    /* Wait BUSY(KS_STS[2]) to 0 */
    u32TimeOutCnt = KS_TIMEOUT;
    while(KS->STS & KS_STS_BUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for Key Store busy flag time-out!\n");
            return -1;
        }
    }

    /* Read key from KS_KEY[0]~[7] registers if EIF(KS_STS[1]) is 1 */
    u32TimeOutCnt = KS_TIMEOUT;
    while(!(KS->STS & KS_STS_EIF_Msk))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for Key Store time-out!\n");
            return -1;
        }
    }

    if(KS->REMAIN != 0x8001000)
    {
        printf(" Fail! Remaining 0x%X space not clear for SRAM.\n", KS->REMAIN & 0x1FFF);
    }

    return 0;
}

void TAMPER_IRQHandler(void)
{
    uint32_t i;
    uint32_t u32FlagStatus;
    uint32_t u32TimeOutCnt;

    /* Tamper interrupt occurred */
    if(TAMPER_GET_INT_FLAG())
    {
        /* Get tamper interrupt status */
        u32FlagStatus = TAMPER_GET_INT_STATUS();

        for(i = 0; i < 6; i++)
        {
            if(u32FlagStatus & (0x1UL << (i + TAMPER_INTSTS_TAMP0IF_Pos)))
                printf(" Tamper %d Detected!!\n", i);
        }

        if(s_u8Option == '0')
        {
            /* Clear tamper interrupt status */
            TAMPER_CLR_INT_STATUS(u32FlagStatus);

            printf(" System wake-up!\n");
        }
        else if(s_u8Option == '1')
        {
            /* Clear tamper interrupt status */
            TAMPER_CLR_INT_STATUS(u32FlagStatus);

            printf(" Check keys are all zero ...");
            if( KS_TRIG() == 0 )
                printf(" Pass!\n");
        }

        /* To check if all the debug messages are finished */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(IsDebugFifoEmpty() == 0)
            if(--u32TimeOutCnt == 0) break;
        SYS_ResetChip();
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

    /* Enable TAMPER module clock */
    CLK_EnableModuleClock(TAMPER_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set multi-function pins for TAMPER */
    SYS->GPF_MFPH &= ~(TAMPER2_PF8_Msk | TAMPER3_PF9_Msk);
    SYS->GPF_MFPH |= (TAMPER2_PF8 | TAMPER3_PF9);

    /* Set PC multi-function pins for CLKO(PC.13) */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC13MFP_Msk) | SYS_GPC_MFPH_PC13MFP_CLKO;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

int32_t KS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /* Enable key store clock */
    CLK->AHBCLK |= 0x2000;

    /* Key store initialization */
    KS->CTL = KS_CTL_INIT_Msk | KS_CTL_START_Msk;

    /* Waiting for initialization was done */
    u32TimeOutCnt = KS_TIMEOUT;
    while(!(KS->STS & KS_STS_INITDONE_Msk))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for Key Store initialization done time-out!\n");
            return -1;
        }
    }

    return 0;
}

int32_t KeyStoreSRAM(void)
{
    uint32_t u32TimeOutCnt;

    /* Check BUSY(KS_STS[2]), EIF(KS_STS[1]) and SRAMFULL(KS_STS[3]) is 0 */
    if(KS->STS & (KS_STS_BUSY_Msk | KS_STS_EIF_Msk | KS_STS_SRAMFULL_Msk))
    {
        printf("Key store status is wrong!\n");
    }

    /* Configure according key's metadata */
    KS->METADATA = ((0x1 << KS_METADATA_READABLE_Pos) | (0x6 << KS_METADATA_SIZE_Pos) | (0x5 << KS_METADATA_OWNER_Pos) | (0x0 << KS_METADATA_DST_Pos));

    /* Set OPMODE(KS_CTL[3:1]) to 001. */
    KS->CTL = (KS->CTL & (~KS_CTL_OPMODE_Msk)) | (1 << KS_CTL_OPMODE_Pos);

    /* Write key to KS_KEY[0]~[7] registers */
    KS->KEY[0] = 0x12345678;
    KS->KEY[1] = 0x12345678;
    KS->KEY[2] = 0x12345678;
    KS->KEY[3] = 0x12345678;
    KS->KEY[4] = 0x12345678;
    KS->KEY[5] = 0x12345678;
    KS->KEY[6] = 0x12345678;
    KS->KEY[7] = 0x12345678;

    /* Set START(KS_CTL[0]) to 1 */
    KS->CTL |= (1 << KS_CTL_START_Pos);

    /* Wait BUSY(KS_STS[2]) to 0 */
    while(KS->STS & KS_STS_BUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for Key Store busy flag time-out!\n");
            return -1;
        }
    }

    /* Check EIF(KS_STS[1]) is 0 */
    if(KS->STS & KS_STS_EIF_Msk)
    {
        printf("EIF(KS_STS[1]) is not 0.\n");
    }

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Enable FMC ISP function */
    FMC_Open();

    /* Enable APROM update function */
    FMC_ENABLE_AP_UPDATE();

    /* Enable User Configuration update function */
    FMC_ENABLE_CFG_UPDATE();

    /* Enable Tamper Domain */
    if(FMC_Read(FMC_USER_CONFIG_3) == 0x5AA5FFFF)
    {
        FMC_Erase(FMC_USER_CONFIG_3);
        SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
        while(1);
    }

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+--------------------------------------+\n");
    printf("|     Tamper Detection Sample Code     |\n");
    printf("+--------------------------------------+\n\n");

    /* Output selected clock to CKO, CKO Clock = HCLK / 2^(1 + 1) */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 1, 0);

    /* Reset tamper coreblock */
    TAMPER_CORE_RESET();
    TAMPER_CORE_RELEASE();

    printf("# Please connect TAMPER2/3(PF.8/9) pins to High first.\n");
    printf("\nSelect:\n");
    printf("    [0] Wake-up test\n");
    printf("    [1] Clear keys in SRAM of Key Store test\n");

    s_u8Option = (uint8_t)getchar();
    printf("\n");
    printf("Select item [%c]\n", s_u8Option);

    TAMPER_CLR_INT_STATUS(TAMPER_INTSTS_TAMP2IF_Msk | TAMPER_INTSTS_TAMP3IF_Msk);

    TAMPER_IOSEL_TAMPER(TAMPER_TAMPER2_SELECT | TAMPER_TAMPER3_SELECT);

    TAMPER_StaticTamperEnable(TAMPER_TAMPER2_SELECT | TAMPER_TAMPER3_SELECT, TAMPER_TAMPER_HIGH_LEVEL_DETECT,
                              TAMPER_TAMPER_DEBOUNCE_ENABLE);

    /* Enable tamper I/O interrupt */
    TAMPER_EnableInt(TAMPER_INTEN_TAMP2IEN_Msk | TAMPER_INTEN_TAMP3IEN_Msk);
    NVIC_EnableIRQ(TAMPER_IRQn);

    if(s_u8Option == '0')
    {
        printf("# Please connect TAMPER2/3(PF.8/9) pins to Low to generate TAMPER event.\n");
        printf("# Press any key to start test:\n\n");
        getchar();

        /* Enable wake-up function */
        TAMPER_ENABLE_WAKEUP();

        /* System enter to Power-down */
        /* To program PWRCTL register, it needs to disable register protection first. */
        SYS_UnlockReg();
        printf("# System enter to power-down mode ...\n");
        /* To check if all the debug messages are finished */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(IsDebugFifoEmpty() == 0)
            if(--u32TimeOutCnt == 0) break;
        CLK_PowerDown();
    }
    else if(s_u8Option == '1')
    {
        /* Enable to trigger Key Store */
        TAMPER_ENABLE_KS_TRIG();

        /* Unlock protected registers */
        SYS_UnlockReg();

        /* Init Key Store */
        if( KS_Init() <0 ) goto lexit;

        printf("# Write keys ...");
        if( KeyStoreSRAM() < 0 ) goto lexit;
        printf(" Done!\n\n");

        printf("# Please connect TAMPER2/3(PF.8/9) pins to Low to generate TAMPER event.\n");
        printf("# Press any key to start test:\n\n");
        getchar();

        printf("# Check keys in SRAM of Key Store when tamper event occurred:\n");

        /* Wait for tamper event interrupt happened */
        while(1);
    }

lexit:

    while(1);

}
