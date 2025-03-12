/***************************************************************************//**
 * @file     fmc_user.h
 * @brief    M2354 series FMC driver header file
 * @version  2.0.0
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef FMC_USER_H
#define FMC_USER_H

#include "targetdev.h"

#define CONFIG0         FMC_CONFIG_BASE
#define CONFIG1         (FMC_CONFIG_BASE+4)

#define ISPGO           0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Define parameter                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define _FMC_ENABLE_CFG_UPDATE()   (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk) /*!< Enable CONFIG Update Function  */
#define _FMC_DISABLE_CFG_UPDATE()  (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk) /*!< Disable CONFIG Update Function */


int FMC_WriteUser(uint32_t u32Addr, uint32_t u32Data);
int FMC_ReadUser(uint32_t u32Addr, uint32_t *pu32Data);
int FMC_EraseUser(uint32_t u32Addr);
void ReadData(uint32_t u32AddrStart, uint32_t u32AddrEnd, uint32_t *pu32Data);
void WriteData(uint32_t u32AddrStart, uint32_t u32AddrEnd, uint32_t *pu32Data);
int EraseAP(uint32_t u32AddrStart, uint32_t u32TotalSize);
void UpdateConfig(uint32_t *pu32Data, uint32_t *pu32Res);

void GetDataFlashInfo(uint32_t *pu32Addr, uint32_t *pu32Size);

#endif

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
