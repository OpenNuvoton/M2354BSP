/**************************************************************************//**
 * @file     NuBL32_main.c
 * @version  V1.00
 * @brief    Executing in NuBL32 then jump to execute in Non-secure NuBL33 F/W.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <arm_cmse.h>
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "partition_M2354.h"

#include "NuBL_common.h"
#include "ota.h"



#define NEXT_BOOT_BASE  (NUBL33_FW_BASE | NS_OFFSET)
#define JUMP_HERE       0xe7fee7ff      /* Instruction Code of "B ." */

extern __NONSECURE_ENTRY int32_t OTA_Init(uint32_t u32HSI, ISP_INFO_T *pISPInfo);
extern __NONSECURE_ENTRY int8_t OTA_TaskProcess(void);
extern __NONSECURE_ENTRY uint8_t OTA_SysTickProcess(uint32_t u32Ticks);
extern __NONSECURE_ENTRY void OTA_WiFiProcess(void);
extern __NONSECURE_ENTRY void OTA_SDH_Process(void);
extern __NONSECURE_ENTRY int32_t OTA_GetBLxFwVer(uint32_t * pu32FwVer, uint8_t u8Mode);
extern __NONSECURE_ENTRY int32_t OTA_ForceUpdate(void);

extern __NONSECURE_ENTRY void BL2_ECC_ISR(void);
__NONSECURE_ENTRY
void ShowCountersInNuBL32(uint32_t *in)__attribute__((noreturn));

/* typedef for NonSecure callback functions */
typedef __NONSECURE_CALL int32_t (*NonSecure_funcptr)(uint32_t);


static volatile ISP_INFO_T     g_ISPInfo = {0};


__NONSECURE_ENTRY
void WdtResetCnt(void);
__NONSECURE_ENTRY
void BL32_OTA_Start(void);
__NONSECURE_ENTRY
int32_t BL32_GetBL33FwVer(uint32_t * pu32FwVer);
void SysTick_Handler(void);
void CRPT_IRQHandler(void);
void UART3_IRQHandler(void);
void GPA_IRQHandler(void);
void Nonsecure_Init(void);
void SYS_Init(void);
void UART_Init(void);
void UART3_Init(void);
void GPIO_init(void);
/*----------------------------------------------------------------------------
  Secure function for NonSecure callbacks exported to NonSecure application
  Must place in Non-secure Callable
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY
void ShowCountersInNuBL32(uint32_t *in)
{
    static uint32_t i = 0;

    printf("\n*** Non-secure callable API in NuBL32 (NuBL33 SRAM address: 0x%08x) ***\n", (uint32_t)in);
    printf("Counters:\n");
    while(1)
    {
        CLK_SysTickDelay(200000);
        CLK_SysTickDelay(200000);
        CLK_SysTickDelay(200000);
        printf("   %d\r", i++);
    }
}

__NONSECURE_ENTRY
void WdtResetCnt(void)
{
    WDT_RESET_COUNTER();
}

__NONSECURE_ENTRY
void BL32_OTA_Start()
{
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    printf("BL32_OTA_Start\n");
    if(OTA_TaskProcess() == 0)
    {
        /* upgrade done, set OTA status to 1 */
        FMC_Erase(OTA_STATUS_BASE);
        FMC_Write(OTA_STATUS_BASE, 0x1UL);
        printf("check OTA status: %d\n", FMC_Read(OTA_STATUS_BASE));
        if(FMC_Read(OTA_STATUS_BASE) == 1)
        {
            printf("BL32_OTA_done\n");
//            SYS_ResetChip();
//            while(1);
        }

    }
}

__NONSECURE_ENTRY
int32_t BL32_GetBL33FwVer(uint32_t * pu32FwVer)
{
    /* get BL33 firmware version */
    return OTA_GetBLxFwVer(pu32FwVer, 1);
}

/*----------------------------------------------------------------------------
  SysTick IRQ Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t u32Ticks;

//    printf("Tick\n");
    if(OTA_SysTickProcess(u32Ticks))
        u32Ticks = 0;
    else
        u32Ticks++;

}

void CRPT_IRQHandler()
{
    printf("CRPT_IRQHandler in NuBL32\n");
    BL2_ECC_ISR();
}

void UART3_IRQHandler(void)
{
//    printf("UART3\n");
    OTA_WiFiProcess();
}


/**
 * @brief       GPIO PA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PA default IRQ, declared in startup_M2351.s.
 */
void GPA_IRQHandler(void)
{
    int32_t i32Status;
    /* To check if PA.3 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT3))
    {
        GPIO_CLR_INT_FLAG(PA, BIT3);
        printf("PA.3 INT occurred.\n");
        i32Status = OTA_ForceUpdate();
        if(i32Status)
        {
            printf("Force update firmware was failed(0x%x).\n", i32Status);
        }
    }
    else
    {
        /* Un-expected interrupt. Just clear all PA interrupts */
        PA->INTSRC = PA->INTSRC;
        printf("Un-expected interrupts.\n");
    }
}

void Nonsecure_Init(void)
{
    NonSecure_funcptr fp;

    /* SCB_NS.VTOR points to the Non-secure vector table base address. */
    SCB_NS->VTOR = NEXT_BOOT_BASE;

    /* 1st Entry in the vector table is the Non-secure Main Stack Pointer. */
    __TZ_set_MSP_NS(*((uint32_t *)SCB_NS->VTOR));      /* Set up MSP in Non-secure code */

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    fp = ((NonSecure_funcptr)(*(((uint32_t *)SCB_NS->VTOR) + 1)));

    /* Clear the LSB of the function address to indicate the function-call
       will cause a state switch from Secure to Non-secure */
    fp = cmse_nsfptr_create(fp);

    /* Check if the Reset_Handler address is in Non-secure space */
    if(cmse_is_nsfptr(fp) && (((uint32_t)fp & 0xf0000000) == 0x10000000))
    {
        printf("Execute non-secure code ...\n");
        fp(0); /* Non-secure function call */
    }
    else
    {
        /* Something went wrong */
        printf("No code in non-secure region!\n");
        printf("CPU will halted at non-secure state\n");

        /* Set nonsecure MSP in nonsecure region */
        __TZ_set_MSP_NS(NON_SECURE_SRAM_BASE + 512);

        /* Try to halted in non-secure state (SRAM) */
        M32(NON_SECURE_SRAM_BASE) = JUMP_HERE;
        fp = (NonSecure_funcptr)(NON_SECURE_SRAM_BASE + 1);
        fp(0);

        while(1) {}
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

void UART3_Init()
{
    CLK->APBCLK0 |= CLK_APBCLK0_UART3CKEN_Msk;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_UART3SEL_Msk)) | CLK_CLKSEL2_UART3SEL_HIRC;

    UART3->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART3->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

    /* Set multi-function pins for UART3 RXD and TXD */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~(UART3_RXD_PD0_Msk | UART3_TXD_PD1_Msk))) | UART3_RXD_PD0 | UART3_TXD_PD1;
}

void GPIO_init(void)
{
    /* Configure PA.3 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PA, BIT3, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 3, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPA_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PA, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT3);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t    count = 0;
    uint32_t    u32FwVer = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n[HCLK %d Hz] \n", SystemCoreClock);
    printf("+--------------------------------+\n");
    printf("|    M2354 NuBL32 Sample Code    |\n");
    printf("+--------------------------------+\n\n");
    if(OTA_GetBLxFwVer((uint32_t *)&u32FwVer, 0) == 0)
        printf("NuBL32 Firmware Ver: 0x%08x\n\n", u32FwVer);
    else
        printf("NuBL32 Firmware Ver: N/A\n\n");
    CLK_SysTickDelay(200000);

    {
        extern const FW_INFO_T g_FWinfoInitial;
        uint32_t cfg = g_FWinfoInitial.mData.u32AuthCFGs;
        printf("\n[AuthCFG: 0x%08x]\n", cfg);
    }

    /* To check if system has been reset by WDT time-out reset or not */
    if(WDT_GET_RESET_FLAG() == 1)
    {
        WDT_CLEAR_RESET_FLAG();

        printf("*** System has been reset by WDT time-out event ***\n\n");
    }

    GPIO_init();

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    printf("[Secure regon boundary: 0x%x]\n\n", SCU->FNSADDR);

    printf("OTA and WIfi init...\n");
    /* init OTA */
    OTA_Init(__HSI, (ISP_INFO_T *)((uint32_t)&g_ISPInfo));

    UART3_Init();

    SystemCoreClockUpdate();
    /* Generate Systick interrupt each 1 ms */
    SysTick_Config(SystemCoreClock / 1000);

//    BL32_OTA_Start();

    /* Unlock protected registers */
    SYS_UnlockReg();
    while(1)
    {
        printf("%d  \r", count);

        count++;
        if(count++ == (5 * 3))
        {
            printf("\n\n");
            Nonsecure_Init(); /* Jump to Non-secure code */
        }
    }
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
