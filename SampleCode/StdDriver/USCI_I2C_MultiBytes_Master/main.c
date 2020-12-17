
/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 19/12/25 2:06p $
 * @brief    Show a Master how to access Slave.
 *           This sample code needs to work with USCI_I2C_Slave.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8DeviceAddr;

void SYS_Init(void);
void UI2C0_Init(uint32_t u32ClkSpeed);


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

    /* Enable UI2C0 peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Set PA multi-function pins for UI2C0_SDA(PA.10) and UI2C0_SCL(PA.11) */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA11MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA10MFP_USCI0_DAT0 | SYS_GPA_MFPH_PA11MFP_USCI0_CLK);
}

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);

    /* Get USCI_I2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x15, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x35, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */

    /* Set USCI_I2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x01);                    /* Slave Address Mask : 0x1 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x04);                    /* Slave Address Mask : 0x4 */
}

int main(void)
{
    uint16_t u16i;
    uint8_t au8TxBuf[256] = {0}, au8rDataBuf[256] = {0};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with multi bytes write
        and multi bytes read operations, and check if the read data is equal to the programmed data.
    */

    printf("+---------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for Master access          |\n");
    printf("|  7-bit address Slave. Needs to work with USCI_I2C_Slave |\n");
    printf("|  sample code                                            |\n");
    printf("|                                                         |\n");
    printf("|  UI2C0(Master)  <----> UI2C0(Slave)                     |\n");
    printf("+---------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a Master\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PA.10), UI2C0_SCL(PA.11)\n");

    /* Init UI2C0 bus bard rate */
    UI2C0_Init(100000);

    /* Slave address */
    g_u8DeviceAddr = 0x15;

    for(u16i = 0; u16i < 256; u16i++)
    {
        au8TxBuf[u16i] = (uint8_t) u16i + 3;
    }

    for(u16i = 0; u16i < 256; u16i += 32)
    {
        /* Write 32 bytes data to Slave */
        while(UI2C_WriteMultiBytesTwoRegs(UI2C0, g_u8DeviceAddr, u16i, &au8TxBuf[u16i], 32) < 32);
    }

    printf("Multi bytes Write access Pass.....\n");

    printf("\n");

    /* Use Multi Bytes Read from Slave (Two Registers) */
    while(UI2C_ReadMultiBytesTwoRegs(UI2C0, g_u8DeviceAddr, 0x0000, au8rDataBuf, 256) < 256);

    /* Compare TX data and RX data */
    for(u16i = 0; u16i < 256; u16i++)
    {
        if(au8TxBuf[u16i] != au8rDataBuf[u16i])
            printf("Data compare fail... R[%d] Data: 0x%X\n", u16i, au8rDataBuf[u16i]);
    }
    printf("Multi bytes Read access Pass.....\n");

    while(1);
}
/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/


