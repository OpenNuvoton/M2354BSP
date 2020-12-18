/**************************************************************************//**
 * @file     NuBL2.h
 * @version  V3.00
 * @brief    NuBL2 header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define NUBL32_FW_INFO_BASE     0x00038000ul    // 224K   
#define NUBL33_FW_INFO_BASE     0x10078000ul    // 480K (Non-secure flash region)  


/**
  * @details    ECC public key structure
  */
typedef struct
{
    uint32_t        au32Key0[8];    /* 256-bits */
    uint32_t        au32Key1[8];    /* 256-bits */
} __attribute__((packed)) ECC_PUBKEY_T;

/**
  * @details    ECC ECDSA signature structure
  */
typedef struct
{
    uint32_t        au32R[8];   /* 256-bits */
    uint32_t        au32S[8];   /* 256-bits */
} __attribute__((packed)) ECDSA_SIGN_T;

/**
  * @details    Firmware region structure
  */
typedef struct
{
    uint32_t    u32Start;   /* 32-bits */
    uint32_t    u32Size;    /* 32-bits */
} __attribute__((packed)) FW_REGION_T;

/**
  * @details    Metadata structure
  */
typedef struct
{
    uint32_t        u32AuthCFGs;        /* 32-bits */
    uint32_t        u32FwRegionLen;     /* 32-bits */
    FW_REGION_T     au32FwRegion[1];    /* (8*1) bytes */
    uint32_t        u32ExtInfoLen;      /* 32-bits */
    uint32_t        au32ExtInfo[3];     /* 12-bytes */
} __attribute__((packed)) METADATA_T;

/**
  * @details    FWINFO structure
  */
typedef struct
{
    ECC_PUBKEY_T    pubkey;             /* 64-bytes ECC-256 public key (256-bits + 256-bits) */

    METADATA_T      mData;              /* Includes ID Hash Configuration, FW regions and Extend Info */

    uint32_t        au32FwHash[8];      /* 32-bytes (256-bits) */

    ECDSA_SIGN_T    sign;               /* 64-bytes (256-bits R + 256-bits S) */
} __attribute__((packed)) FW_INFO_T;


int32_t VerifyNuBL3x(uint32_t *pu32FwInfo, uint32_t u32InfoBase);
extern const uint32_t g_InitialFWInfo[]; // A global variable to store NuBL2 FWINFO address, declared in FwInfo.c

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
