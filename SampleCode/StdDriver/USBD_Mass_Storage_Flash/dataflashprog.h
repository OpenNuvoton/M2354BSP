/***************************************************************************//**
 * @file     dataflashprog.h
 * @brief    M2354 series data flash programming driver header
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DATA_FLASH_PROG_H__
#define __DATA_FLASH_PROG_H__

#define MASS_STORAGE_OFFSET       0x00080000  /* To avoid the code to write APROM */
#define DATA_FLASH_STORAGE_SIZE   (512*1024)  /* Configure the DATA FLASH storage u32Size. To pass USB-IF MSC Test, it needs > 64KB */
#define FLASH_PAGE_SIZE           2048
#define BUFFER_PAGE_SIZE          512

void DataFlashWrite(uint32_t u32Addr, uint32_t u32Size, uint32_t u32Buffer);
void DataFlashRead(uint32_t u32Addr, uint32_t u32Size, uint32_t u32Buffer);

#endif  /* __DATA_FLASH_PROG_H__ */
