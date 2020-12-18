/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * @brief    Use M2354 as USB Host to perform SecureISP.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "usbh_lib.h"
#include "usbh_hid.h"


HID_DEV_T   *g_hid_list[CONFIG_HID_MAX_DEV];
uint8_t     g_buff_pool[1024] __attribute__((aligned(32)));

extern int32_t Process_USBHCommand(HID_DEV_T *hdev);

volatile uint32_t  g_tick_cnt;
void SysTick_Handler(void)
{
    g_tick_cnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    g_tick_cnt = 0;
    if(SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while(1);
    }
}

uint32_t get_ticks()
{
    return g_tick_cnt;
}

void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from XTL_12M. Prescale 12
     */
    /* TIMER0 clock from HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    TIMER0->CTL = 0;        /* disable timer */
    TIMER0->INTSTS = (TIMER_INTSTS_TIF_Msk | TIMER_INTSTS_TWKF_Msk);   /* write 1 to clear for safety */
    TIMER0->CMP = usec;
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;

    while(!TIMER0->INTSTS) {}
}

void  dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while(nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for(i = 0; (i < 16) && (nBytes > 0); i++)
        {
            printf("%02x ", pucBuff[nIdx + i]);
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

int  is_a_new_hid_device(HID_DEV_T *hdev)
{
    int    i;
    for(i = 0; i < CONFIG_HID_MAX_DEV; i++)
    {
        if((g_hid_list[i] != NULL) && (g_hid_list[i] == hdev) &&
                (g_hid_list[i]->uid == hdev->uid))
            return 0;
    }
    return 1;
}

void update_hid_device_list(HID_DEV_T *hdev)
{
    int  i = 0;
    memset(g_hid_list, 0, sizeof(g_hid_list));
    while((i < CONFIG_HID_MAX_DEV) && (hdev != NULL))
    {
        g_hid_list[i++] = hdev;
        hdev = hdev->next;
    }
}

int  init_hid_device(HID_DEV_T *hdev)
{
    uint8_t   *data_buff;
    int       i, ret;

    data_buff = (uint8_t *)((uint32_t)g_buff_pool);

    printf("\n\n==================================\n");
    printf("  Init HID device : 0x%x\n", (int)hdev);
    printf("  VID: 0x%x, PID: 0x%x\n\n", hdev->idVendor, hdev->idProduct);

    ret = usbh_hid_get_report_descriptor(hdev, data_buff, 1024);
    if(ret > 0)
    {
        printf("\nDump report descriptor =>\n");
        dump_buff_hex(data_buff, ret);
    }

    return 0;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as PCLK and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable USBH module clock, default clock source is from PLL */
    CLK_EnableModuleClock(USBH_MODULE);

    /* USB Host desired input clock is 48 MHz. Set as PLL divided by 2 (96/2 = 48) */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBDIV_Msk) | CLK_CLKDIV0_USB(2);

    /* Enable USBD and OTG clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk | CLK_APBCLK0_OTGCKEN_Msk;

    /* Set OTG as USB Host role */
    SYS->USBPHY = SYS_USBPHY_OTGPHYEN_Msk | SYS_USBPHY_SBO_Msk | (0x1 << SYS_USBPHY_USBROLE_Pos);

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15     */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB15MFP_Msk) | SYS_GPB_MFPH_PB15MFP_USB_VBUS_EN;

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.14   */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) | SYS_GPB_MFPH_PB14MFP_USB_VBUS_ST;

    /* USB 1.1 port multi-function pin VBUS, D+, D-, and ID pins */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk |
                       SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA12MFP_USB_VBUS | SYS_GPA_MFPH_PA13MFP_USB_D_N |
                     SYS_GPA_MFPH_PA14MFP_USB_D_P | SYS_GPA_MFPH_PA15MFP_USB_OTG_ID;
}

uint32_t CLK_GetUSBFreq(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Get USB Peripheral Clock                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* USB Peripheral clock = PLL_CLOCK/USBDIV+1) */
    return CLK_GetPLLClockFreq() / (((CLK->CLKDIV0 & CLK_CLKDIV0_USBDIV_Msk) >> CLK_CLKDIV0_USBDIV_Pos) + 1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                        Main function                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
int main()
{
    int          i, ret;
    HID_DEV_T    *hdev, *hdev_list;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    enable_sys_tick(100);

    printf("\n\n");
    printf("System clock:   %d Hz.\n", SystemCoreClock);
    printf("USB Host clock: %d Hz.\n", CLK_GetUSBFreq());
    printf("+-----------------------------------------------+\n");
    printf("|                                               |\n");
    printf("|        USB Host Secure ISP Sample Code        |\n");
    printf("|                                               |\n");
    printf("+-----------------------------------------------+\n");
    printf("Wait until any HID devices connected...\n\n");

    usbh_core_init();
    usbh_hid_init();
    usbh_memory_used();

    memset(g_hid_list, 0, sizeof(g_hid_list));

    while(1)
    {
        if(usbh_pooling_hubs())              /* USB Host port detect polling and management */
        {
            usbh_memory_used();              /* print out USB memory allocating information */

            printf("\n Has hub events.\n");
            hdev_list = usbh_hid_get_device_list();
            hdev = hdev_list;
            while(hdev != NULL)
            {
                if(is_a_new_hid_device(hdev))
                {
                    init_hid_device(hdev);
                    Process_USBHCommand(hdev);
                }
                hdev = hdev->next;
            }

            update_hid_device_list(hdev_list);
            usbh_memory_used();
        }
    }

    while(1) {}
}
