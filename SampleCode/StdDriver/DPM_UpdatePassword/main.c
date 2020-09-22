/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to update and compare Secure DPM password.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_96MHZ

void SYS_Init(void);
void UART0_Init(void);

void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable SRAM module clock */
    CLK_EnableModuleClock(SRAM0_MODULE);
    CLK_EnableModuleClock(SRAM1_MODULE);
    CLK_EnableModuleClock(SRAM2_MODULE);

    /* Enable GPIO module clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
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
    printf("Secure region debug is %s.\n", DPM_GetDebugDisable(SECURE_DPM)? "Disabled": "Enabled");     
    printf("Secure region debug is %s.\n\n", DPM_GetDebugLock(SECURE_DPM)? "Locked": "not Locked");      
    
    /* Update password */    
    printf("Update Secure DPM password ... ");

    if(DPM_SetPasswordUpdate(SECURE_DPM, au32Pwd_5)==0)
    {
        printf("Password update has reached maximum time. Please erase chip.\n");
        while(1);
    }
    else
    {
        printf("OK.\n");
    }        
    
    /* Compare password fail */   
    printf("Compare with wrong password ... ");
    
    if(DPM_SetPasswordCompare(SECURE_DPM, au32Pwd_A)==0)
    {
        if(DPM_GetPasswordErrorFlag(SECURE_DPM)==1)
        {
            printf("password is wrong.\n");
            DPM_ClearPasswordErrorFlag(SECURE_DPM);  
        }
        else
        {
            printf("OK.\n");
        }
    }
    else
    {
        printf("Password compare has reached maximum time. Please erase chip.\n");
        while(1);           
    }

    /* Compare password pass */   
    printf("Compare with correct password ... ");
    
    if(DPM_SetPasswordCompare(SECURE_DPM, au32Pwd_5)==0)
    {
        if(DPM_GetPasswordErrorFlag(SECURE_DPM)==1)
        {
            printf("password is wrong.\n");
            DPM_ClearPasswordErrorFlag(SECURE_DPM);  
        }
        else
        {
            printf("OK.\n");
        }
    }
    else
    {
        printf("\n\n\tPassword compare has reached maximum time. Please erase chip.");      
        while(1);           
    }     

    while(1);
    
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
