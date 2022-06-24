/**************************************************************************//**
 * @file     NuBL32_main.c
 * @version  V1.00
 * @brief    Demonstrate NuBL33. (Non-secure code)
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include "NuBL_common.h"


int32_t NonSecure_LED_On(uint32_t num);
int32_t NonSecure_LED_Off(uint32_t num);
void SysTick_Handler(void);

/* Non-secure Callable function of NuBL32 */
extern void ShowCountersInNuBL32(uint32_t *in);
extern void BL32_OTA_Start(void);
extern void WdtResetCnt(void);
extern int32_t BL32_GetBL33FwVer(uint32_t * pu32FwVer);

typedef int32_t (*funcptr)(uint32_t);


extern int32_t Secure_func(void);

/*----------------------------------------------------------------------------
  NonSecure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
extern int32_t Secure_LED_On_callback(int32_t (*)(uint32_t));
extern int32_t Secure_LED_Off_callback(int32_t (*)(uint32_t));
extern int32_t Secure_LED_On(uint32_t num);
extern int32_t Secure_LED_Off(uint32_t num);

/*----------------------------------------------------------------------------
  NonSecure functions used for callbacks
 *----------------------------------------------------------------------------*/
int32_t NonSecure_LED_On(uint32_t u32Num)
{
    (void) u32Num;
    printf("Nonsecure LED On call by Secure\n");
	#if defined( NUMAKER_BOARD )
    PD3_NS = 0;
	#endif
    return 0;
}

int32_t NonSecure_LED_Off(uint32_t u32Num)
{
    (void) u32Num;
    printf("Nonsecure LED Off call by Secure\n");
	#if defined( NUMAKER_BOARD )
    PD3_NS = 1;
	#endif
    return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/* SysTick IRQ Handler                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t ticks;
    static uint32_t u32Ticks;

    if(u32Ticks > 800) /* 8s*/
    {
        WdtResetCnt();
        u32Ticks = 0;
    }
    u32Ticks++;

    switch(ticks++)
    {
        case   0:
            Secure_LED_On(6u);
            break;
        case 400:
            Secure_LED_Off(6u);
            break;
        default:
            if(ticks > 500)
            {
                ticks = 0;
            }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t    u32FwVer = 0;
    uint32_t cfg = g_FWinfoInitial.mData.u32AuthCFGs;

    printf("\n");
    printf("+--------------------------------------------+\n");
    printf("|    M2354 NuBL33(Non-secure) Sample Code    |\n");
    printf("+--------------------------------------------+\n\n");
    printf("\n[AuthCFG: 0x%08x]\n", cfg);

    if(BL32_GetBL33FwVer((uint32_t *)&u32FwVer) == 0)
        printf("NuBL33 Firmware Ver: 0x%08x\n\n", u32FwVer);
    else
        printf("NuBL33 Firmware Ver: N/A\n\n");
#if defined( NUMAKER_BOARD )
    /* Init PD.3 for Nonsecure LED control */
    GPIO_SetMode(PD_NS, BIT3, GPIO_MODE_OUTPUT);
#endif
    /* register NonSecure callbacks in Secure application */
    Secure_LED_On_callback(&NonSecure_LED_On);
    Secure_LED_Off_callback(&NonSecure_LED_Off);

    /* Generate Systick interrupt each 10 ms */
//    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 100);

    BL32_OTA_Start();
//    ShowCountersInNuBL32(&temp);

    while(1) {}
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
