/****************************************************************************
 * @file     board-config.h
 * @version  V1.10
 * @brief    Board configuration
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 PF_6
#define RADIO_TCXO_EN                               PA_8

#define RADIO_MOSI                                  PC_11
#define RADIO_MISO                                  PC_12
#define RADIO_SCLK                                  PC_10
#define RADIO_NSS                                   PC_9

#define RADIO_DIO_0                                 PF_7
#define RADIO_DIO_1                                 PF_8
#define RADIO_DIO_2                                 PF_9
#define RADIO_DIO_3                                 PF_10
#define RADIO_DIO_4                                 PF_11
#define RADIO_DIO_5                                 PD_12

#define LED_1                                       NC
#define LED_2                                       NC

#define BOARD_TCXO_WAKEUP_TIME                      5

#define SPI3_MFP_MOSI_VALUE		(SYS_GPC_MFPH_PC11MFP_SPI3_MOSI)						
#define SPI3_MFP_MISO_VALUE		(SYS_GPC_MFPH_PC12MFP_SPI3_MISO)							
#define SPI3_MFP_CLK_VALUE		(SYS_GPC_MFPH_PC10MFP_SPI3_CLK)					
#define SPI3_MFP_NSS_VALUE		(SYS_GPC_MFPH_PC9MFP_SPI3_SS)							

#define I2C0_SCL										PC_1
#define I2C0_SDA										PC_0

#define I2C1_SCL										PC_5
#define I2C1_SDA										PC_4

#define I2C0_MFP_SCL_VALUE		(SYS_GPC_MFPL_PC1MFP_I2C0_SCL)						
#define I2C0_MFP_SDA_VALUE		(SYS_GPC_MFPL_PC0MFP_I2C0_SDA)							
#define I2C1_MFP_SCL_VALUE		(SYS_GPC_MFPL_PC5MFP_I2C1_SCL)					
#define I2C1_MFP_SDA_VALUE		(SYS_GPC_MFPL_PC4MFP_I2C1_SDA)							

#define UART0_TX										PB_9
#define UART0_RX										PB_8

#define UART1_TX										PB_3
#define UART1_RX										PB_2

#define UART0_MFP_TX_VALUE		(SYS_GPB_MFPH_PB9MFP_UART0_TXD)						
#define UART0_MFP_RX_VALUE		(SYS_GPB_MFPH_PB8MFP_UART0_RXD)							
#define UART1_MFP_TX_VALUE		(SYS_GPB_MFPL_PB3MFP_UART1_TXD)					
#define UART1_MFP_RX_VALUE		(SYS_GPB_MFPL_PB2MFP_UART1_RXD)							

#define FMC_ADDR_END		(256 * 1024)							//256K Bytes
#define FACTORY_EEPROM_SIZE	(1 * 1024)								//1K size	
#define FACTORY_EEPROM_BASE	(FMC_ADDR_END - FACTORY_EEPROM_SIZE)	
#define DATA_EEPROM_END		FACTORY_EEPROM_BASE
#define DATA_EEPROM_SIZE	(3 * 1024)								//3K size
#define DATA_EEPROM_BASE	(DATA_EEPROM_END - DATA_EEPROM_SIZE)	


#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed. 
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  //#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */


#endif // __BOARD_CONFIG_H__
	