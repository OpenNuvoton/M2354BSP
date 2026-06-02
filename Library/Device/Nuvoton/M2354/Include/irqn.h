/**************************************************************************//**
 * @file     irqn.h
 * @version  V1.00
 * @brief    IRQ number definition for M2354
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __IRQN_H__
#define __IRQN_H__

/* -------------------  Interrupt Number Definition  ------------------------------ */

typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn       = -14,      /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn            = -13,      /*!< 3 Cortex-M23 Hard Fault Interrupt                    */
    SVCall_IRQn               = -5,       /*!< 11 Cortex-M23 SV Call Interrupt                      */
    PendSV_IRQn               = -2,       /*!< 14 Cortex-M23 Pend SV Interrupt                      */
    SysTick_IRQn              = -1,       /*!< 15 Cortex-M23 System Tick Interrupt                  */

    /******  ARMIKMCU Swift specific Interrupt Numbers ************************************************/

    BOD_IRQn                      = 0,        /*!< Brown Out detection Interrupt                    */
    IRC_IRQn                      = 1,        /*!< Internal RC Interrupt                            */
    PWRWU_IRQn                    = 2,        /*!< Power Down Wake Up Interrupt                     */
    RAMPE_IRQn                    = 3,        /*!< SRAM parity check failed Interrupt               */
    CKFAIL_IRQn                   = 4,        /*!< Clock failed Interrupt                           */
    ISP_IRQn                      = 5,        /*!< FMC ISP Interrupt                                */
    RTC_IRQn                      = 6,        /*!< Real Time Clock Interrupt                        */
    RTC_TAMPER_IRQn               = 7,        /*!< RTC Tamper detection Interrupt                   */
    WDT_IRQn                      = 8,        /*!< Watchdog Timer Interrupt                         */
    WWDT_IRQn                     = 9,        /*!< Window Watchdog Timer Interrupt                  */
    EINT0_IRQn                    = 10,       /*!< External Input 0 Interrupt                       */
    EINT1_IRQn                    = 11,       /*!< External Input 1 Interrupt                       */
    EINT2_IRQn                    = 12,       /*!< External Input 2 Interrupt                       */
    EINT3_IRQn                    = 13,       /*!< External Input 3 Interrupt                       */
    EINT4_IRQn                    = 14,       /*!< External Input 4 Interrupt                       */
    EINT5_IRQn                    = 15,       /*!< External Input 5 Interrupt                       */
    GPA_IRQn                      = 16,       /*!< GPIO Port A Interrupt                            */
    GPB_IRQn                      = 17,       /*!< GPIO Port B Interrupt                            */
    GPC_IRQn                      = 18,       /*!< GPIO Port C Interrupt                            */
    GPD_IRQn                      = 19,       /*!< GPIO Port D Interrupt                            */
    GPE_IRQn                      = 20,       /*!< GPIO Port E Interrupt                            */
    GPF_IRQn                      = 21,       /*!< GPIO Port F Interrupt                            */
    QSPI0_IRQn                    = 22,       /*!< QSPI0 Interrupt                                  */
    SPI0_IRQn                     = 23,       /*!< SPI0 Interrupt                                   */
    BRAKE0_IRQn                   = 24,       /*!< BRAKE0 Interrupt                                 */
    EPWM0_P0_IRQn                 = 25,       /*!< EPWM0P0 Interrupt                                */
    EPWM0_P1_IRQn                 = 26,       /*!< EPWM0P1 Interrupt                                */
    EPWM0_P2_IRQn                 = 27,       /*!< EPWM0P2 Interrupt                                */
    BRAKE1_IRQn                   = 28,       /*!< BRAKE1 Interrupt                                 */
    EPWM1_P0_IRQn                 = 29,       /*!< EPWM1P0 Interrupt                                */
    EPWM1_P1_IRQn                 = 30,       /*!< EPWM1P1 Interrupt                                */
    EPWM1_P2_IRQn                 = 31,       /*!< EPWM1P2 Interrupt                                */
    TMR0_IRQn                     = 32,       /*!< Timer 0 Interrupt                                */
    TMR1_IRQn                     = 33,       /*!< Timer 1 Interrupt                                */
    TMR2_IRQn                     = 34,       /*!< Timer 2 Interrupt                                */
    TMR3_IRQn                     = 35,       /*!< Timer 3 Interrupt                                */
    UART0_IRQn                    = 36,       /*!< UART 0 Interrupt                                 */
    UART1_IRQn                    = 37,       /*!< UART 1 Interrupt                                 */
    I2C0_IRQn                     = 38,       /*!< I2C 0 Interrupt                                  */
    I2C1_IRQn                     = 39,       /*!< I2C 1 Interrupt                                  */
    PDMA0_IRQn                    = 40,       /*!< Peripheral DMA 0 Interrupt                       */
    DAC_IRQn                      = 41,       /*!< DAC Interrupt                                    */
    EADC0_IRQn                    = 42,       /*!< EADC Source 0 Interrupt                          */
    EADC1_IRQn                    = 43,       /*!< EADC Source 1 Interrupt                          */
    ACMP01_IRQn                   = 44,       /*!< Analog Comparator 0 and 1 Interrupt              */
    EADC2_IRQn                    = 46,       /*!< EADC Source 2 Interrupt                          */
    EADC3_IRQn                    = 47,       /*!< EADC Source 3 Interrupt                          */
    UART2_IRQn                    = 48,       /*!< UART2 Interrupt                                  */
    UART3_IRQn                    = 49,       /*!< UART3 Interrupt                                  */
    SPI1_IRQn                     = 51,       /*!< SPI1 Interrupt                                   */
    SPI2_IRQn                     = 52,       /*!< SPI2 Interrupt                                   */
    USBD_IRQn                     = 53,       /*!< USB device Interrupt                             */
    USBH_IRQn                     = 54,       /*!< USB host Interrupt                               */
    USBOTG_IRQn                   = 55,       /*!< USB OTG Interrupt                                */
    CAN0_IRQn                     = 56,       /*!< CAN0 Interrupt                                   */
    SC0_IRQn                      = 58,       /*!< Smart Card 0 Interrupt                           */
    SC1_IRQn                      = 59,       /*!< Smart Card 1 Interrupt                           */
    SC2_IRQn                      = 60,       /*!< Smart Card 2 Interrupt                           */
    SPI3_IRQn                     = 62,       /*!< SPI3 Interrupt                                   */
    SDH0_IRQn                     = 64,       /*!< SDH0 Interrupt                                   */
    I2S0_IRQn                     = 68,       /*!< I2S0 Interrupt                                   */
    CRPT_IRQn                     = 71,       /*!< CRPT Interrupt                                   */
    GPG_IRQn                      = 72,       /*!< GPIO Port G Interrupt                            */
    EINT6_IRQn                    = 73,       /*!< External Input 6 Interrupt                       */
    UART4_IRQn                    = 74,       /*!< UART4 Interrupt                                  */
    UART5_IRQn                    = 75,       /*!< UART5 Interrupt                                  */
    USCI0_IRQn                    = 76,       /*!< USCI0 Interrupt                                  */
    USCI1_IRQn                    = 77,       /*!< USCI1 Interrupt                                  */
    BPWM0_IRQn                    = 78,       /*!< BPWM0 Interrupt                                  */
    BPWM1_IRQn                    = 79,       /*!< BPWM1 Interrupt                                  */
    I2C2_IRQn                     = 82,       /*!< I2C2 Interrupt                                   */
    QEI0_IRQn                     = 84,       /*!< QEI0 Interrupt                                   */
    QEI1_IRQn                     = 85,       /*!< QEI1 Interrupt                                   */
    ECAP0_IRQn                    = 86,       /*!< ECAP0 Interrupt                                  */
    ECAP1_IRQn                    = 87,       /*!< ECAP1 Interrupt                                  */
    GPH_IRQn                      = 88,       /*!< GPIO Port H Interrupt                            */
    EINT7_IRQn                    = 89,       /*!< External Input 7 Interrupt                       */
    PDMA1_IRQn                    = 98,       /*!< Peripheral DMA 1 Interrupt                       */
    SCU_IRQn                      = 99,       /*!< SCU Interrupt                                    */
    LCD_IRQn                      = 100,      /*!< LCD interrupt                                    */
    TRNG_IRQn                     = 101,      /*!< TRNG interrupt                                   */
    KS_IRQn                       = 109,      /*!< Key Store interrupt                              */
    TAMPER_IRQn                   = 110,      /*!< TAMPER interrupt                                 */
    EWDT_IRQn                     = 111,      /*!< Extra Watchdog Timer interrupt                   */
    EWWDT_IRQn                    = 112,      /*!< Extra Window Watchdog Timer interrupt            */
    NS_ISP_IRQn                   = 113,      /*!< Non-secure FMC ISP interrupt                     */
    TMR4_IRQn                     = 114,      /*!< Timer 4 Interrupt                                */
    TMR5_IRQn                     = 115,      /*!< Timer 5 Interrupt                                */


} IRQn_Type;

#endif /* __IRQN_H__ */