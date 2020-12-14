/****************************************************************************
 * @file     main.c
 * @version  V2.00
 * @brief    To utilize emWin library to demonstrate interactive feature.
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "GUI.h"
#include "WM.h"
#include "FRAMEWIN.h"

#include "M2354TouchPanel.h"

extern volatile GUI_TIMER_TIME OS_TimeMS;

static volatile int s_enable_Touch;

extern int ts_writefile(void);
extern int ts_readfile(void);
int ts_calibrate(int xsize, int ysize);
void ts_test(int xsize, int ysize);
void TMR0_IRQHandler(void);
void MainTask(void)__attribute((noreturn));

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _SYS_Init
*/
static void SYS_Init(void)
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

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* Set EADC clock divider as 12 */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(12));

    /* Enable SPI1 peripheral clock */
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Select SPI1 module clock source as PCLK0 */
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, MODULE_NoMsk);

    /* Enable PDMA0 clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /* Set multi-function pins for EADC channels. */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk)) | EADC0_CH0_PB0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB1MFP_Msk)) | EADC0_CH1_PB1;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk)) | EADC0_CH2_PB2;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB3MFP_Msk)) | EADC0_CH3_PB3;

    /* Disable digital input path of EADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT3 | BIT2 | BIT1 | BIT0);

}

/*********************************************************************
*
*       TMR0_IRQHandler
*/

void TMR0_IRQHandler(void)
{
    int Key;
    OS_TimeMS++;
#if GUI_SUPPORT_TOUCH
    if(OS_TimeMS % 10 == 0)
    {
        if(s_enable_Touch == 1)
        {
            GUI_TOUCH_Exec();
        }
    }
#endif
    if((s_enable_Touch == 1) && (OS_TimeMS % 200 == 0))
    {
        if(PB6 == 0)
        {
            Key = GUI_KEY_ENTER;
            GUI_StoreKeyMsg(Key, 1);
            printf("enter1\n");
        }
        if(PB7 == 0)
        {
            Key = GUI_KEY_TAB;
            GUI_StoreKeyMsg(Key, 1);
            printf("tab\n");
        }
        if (PC0 == 0)
        {
            Key = GUI_KEY_DOWN;
            GUI_StoreKeyMsg(Key, 1);
            printf("down\n");
        }
        if(PC1 == 0)
        {
            Key = GUI_KEY_ENTER;
            GUI_StoreKeyMsg(Key, 1);
            printf("enter2\n");
        }
        if(PC12 == 0)
        {
            Key = GUI_KEY_RIGHT;
            GUI_StoreKeyMsg(Key, 1);
            printf("right\n");
        }
        if(PE6 == 0)
        {
            Key = GUI_KEY_UP;
            GUI_StoreKeyMsg(Key, 1);
            printf("up\n");
        }
        if(PE7 == 0)
        {
            Key = GUI_KEY_LEFT;
            GUI_StoreKeyMsg(Key, 1);
            printf("left\n");
        }
    }

    TIMER_ClearIntFlag(TIMER0);
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

WM_HWIN CreateFramewin(void);

void MainTask(void)
{
    extern GUI_CONST_STORAGE GUI_BITMAP bmM2354_320x240;
    WM_HWIN hWin;
    char     *acVersion = "Nuvoton M2354";
    int32_t i32Scale;

    GUI_Init();

    GUI_SetBkColor(GUI_BLACK);
    GUI_Clear();
#ifdef __DEMO_160x128__
    i32Scale = 500;
#else
    i32Scale = 1000;
#endif
    GUI_DrawBitmapEx(&bmM2354_320x240, 0, 5,0,0, i32Scale, i32Scale);
    GUI_Delay(3000);


    hWin = CreateFramewin();
    FRAMEWIN_SetText(hWin, acVersion);

    while(1)
    {
        GUI_Delay(1000);
    }
}

/*********************************************************************
*
*       main
*/

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    //
    // Init System, IP clock and multi-function I/O
    //
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
    //
    // Init UART to 115200-8n1 for print message
    //
    UART_Open(UART0, 115200);

    s_enable_Touch = 0;
    //
    // Enable Timer0 clock and select Timer0 clock source
    //
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    //
    // Initial Timer0 to periodic mode with 1000Hz
    //
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    //
    // Enable Timer0 interrupt
    //
    TIMER_EnableInt(TIMER0);
    NVIC_SetPriority(TMR0_IRQn, 1);
    NVIC_EnableIRQ(TMR0_IRQn);
    //
    // Start Timer0
    //
    TIMER_Start(TIMER0);
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|               emWin GUI Demo                                |\n");
    printf("+-------------------------------------------------------------+\n");
    printf(" This demo code need to executed on NuMaker board with Nu_TFT v1.1/v1.3 daughter board.\n");
    printf("\n\n");

    GPIO_SetMode(PB, BIT6, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);
    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);
    GPIO_SetMode(PC, BIT1, GPIO_MODE_INPUT);
    GPIO_SetMode(PC, BIT12, GPIO_MODE_INPUT);
    GPIO_SetMode(PE, BIT6, GPIO_MODE_INPUT);
    GPIO_SetMode(PE, BIT7, GPIO_MODE_INPUT);

#if GUI_SUPPORT_TOUCH
    GUI_Init();

    Init_TouchPanel();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    /* SPI flash 256KB + 0x1C marker address */
    if(FMC_Read(__DEMO_TSFILE_ADDR__ + 0x1C) != 0x55AAA55A)
    {
        FMC_ENABLE_AP_UPDATE();
        ts_calibrate(__DEMO_TS_WIDTH__, __DEMO_TS_HEIGHT__);
        // Erase page
        FMC_Erase(__DEMO_TSFILE_ADDR__);
        ts_writefile();
        FMC_DISABLE_AP_UPDATE();
    }
    else
    {
        ts_readfile();
    }

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    s_enable_Touch = 1;

//    ts_test(__DEMO_TS_WIDTH__, __DEMO_TS_HEIGHT__);
#endif

    s_enable_Touch = 1;

    //
    // Start application
    //

    MainTask();

}

/*************************** End of file ****************************/
