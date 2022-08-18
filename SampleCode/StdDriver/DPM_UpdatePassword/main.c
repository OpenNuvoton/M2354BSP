/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to update and compare Secure DPM password.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"



void SYS_Init(void);
void UART0_Init(void);

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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t au32Pwd_5[4] = {0x55555555, 0x55555555, 0x55555555, 0x55555555};
    uint32_t au32Pwd_A[4] = {0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA};
    int32_t i32RetVal;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------+\n");
    printf("|    Secure DPM Update Password    |\n");
    printf("+----------------------------------+\n\n");

    /* Get DPM status */
    printf("Secure DPM manages Secure region debug.\n");
    printf("Get Secure DPM status:\n");
    printf("Secure region debug is %s.\n", DPM_GetDebugDisable(SECURE_DPM) ? "Disabled" : "Enabled");
    printf("Secure region debug is %s.\n\n", DPM_GetDebugLock(SECURE_DPM) ? "Locked" : "not Locked");

    /* Update password */
    printf("Update Secure DPM password ... ");

    i32RetVal = DPM_SetPasswordUpdate(SECURE_DPM, au32Pwd_5);
    if(i32RetVal == 0)
    {
        printf("Password update has reached maximum time. Please erase chip.\n");
        goto lexit;
    }
    else if(i32RetVal == DPM_TIMEOUT_ERR)
    {
        printf("Wait for DPM busy flag cleared time-out!\n");
        goto lexit;
    }
    else
    {
        printf("OK.\n");
    }

    /* Compare password fail */
    printf("Compare with wrong password ... ");

    i32RetVal = DPM_SetPasswordCompare(SECURE_DPM, au32Pwd_A);
    if(i32RetVal == 0)
    {
        if(DPM_GetPasswordErrorFlag(SECURE_DPM) == 1)
        {
            printf("password is wrong.\n");
            DPM_ClearPasswordErrorFlag(SECURE_DPM);
        }
        else if(i32RetVal == DPM_TIMEOUT_ERR)
        {
            printf("Wait for DPM busy flag cleared time-out!\n");
            goto lexit;
        }
        else
        {
            printf("OK.\n");
        }
    }
    else if(i32RetVal == DPM_TIMEOUT_ERR)
    {
        printf("Wait for DPM busy flag cleared time-out!\n");
        goto lexit;
    }
    else
    {
        printf("Password compare has reached maximum time. Please erase chip.\n");
        goto lexit;
    }

    /* Compare password pass */
    printf("Compare with correct password ... ");

    i32RetVal = DPM_SetPasswordCompare(SECURE_DPM, au32Pwd_5);
    if( i32RetVal == 0)
    {
        if(DPM_GetPasswordErrorFlag(SECURE_DPM) == 1)
        {
            printf("password is wrong.\n");
            DPM_ClearPasswordErrorFlag(SECURE_DPM);
        }
        else if(i32RetVal == DPM_TIMEOUT_ERR)
        {
            printf("Wait for DPM busy flag cleared time-out!\n");
            goto lexit;
        }
        else
        {
            printf("OK.\n");
        }
    }
    else if(i32RetVal == DPM_TIMEOUT_ERR)
    {
        printf("Wait for DPM busy flag cleared time-out!\n");
        goto lexit;
    }
    else
    {
        printf("\n\n\tPassword compare has reached maximum time. Please erase chip.");
        goto lexit;
    }

lexit:

    while(1);

}
