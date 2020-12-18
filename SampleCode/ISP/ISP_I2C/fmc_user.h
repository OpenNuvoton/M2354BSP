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


int FMC_WriteUser(unsigned int u32Addr, unsigned int u32Data);
int FMC_ReadUser(unsigned int u32Addr, unsigned int *pu32Data);
int FMC_EraseUser(unsigned int u32Addr);
void ReadData(unsigned int u32AddrStart, unsigned int u32AddrEnd, unsigned int *pu32Data);
void WriteData(unsigned int u32AddrStart, unsigned int u32AddrEnd, unsigned int *pu32Data);
int EraseAP(unsigned int u32AddrStart, unsigned int u32TotalSize);
void UpdateConfig(unsigned int *pu32Data, unsigned int *pu32Res);

void GetDataFlashInfo(uint32_t *pu32Addr, uint32_t *pu32Size);

#endif

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
