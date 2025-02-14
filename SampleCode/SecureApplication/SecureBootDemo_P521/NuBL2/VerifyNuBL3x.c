/**************************************************************************//**
 * @file     VerifyNuBL3x.c
 * @version  V3.00
 * @brief    This source file is used to authenticate the NuBL32.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "partition_M2354.h"
#include "NuBL2.h"

#define DBG_EN      0

void CRPT_IRQHandler(void);
int32_t Cal_SHA512_Flash(uint32_t u32Addr, uint32_t u32Bytes, uint32_t *pu32Digest);
int32_t Cal_SHA512_SRAM(uint32_t u32Addr, uint32_t u32Bytes, uint32_t *pu32Digest);
int32_t VerifyNuBL3x(uint32_t *pu32FwInfo, uint32_t u32InfoBase);

void CRPT_IRQHandler(void)
{
    ECC_DriverISR(CRPT);
}

static void BytesSwap(char *buf, int32_t len)
{
    int32_t i;
    char    tmp;

    for(i = 0; i < (len / 2); i++)
    {
        tmp = buf[len - i - 1];
        buf[len - i - 1] = buf[i];
        buf[i] = tmp;
    }
}

int32_t Cal_SHA512_Flash(uint32_t u32Addr, uint32_t u32Bytes, uint32_t *pu32Digest)
{
    volatile int32_t    i, addr, bytes, data;
    uint32_t u32TimeOutCnt;

    addr = (int32_t)u32Addr;
    bytes = (int32_t)u32Bytes;

    /* Reset CRYPTO module */
    SYS_ResetModule(CRPT_MODULE);

    CRPT->HMAC_CTL = (SHA_MODE_SHA512 << CRPT_HMAC_CTL_OPMODE_Pos) | CRPT_HMAC_CTL_INSWAP_Msk | CRPT_HMAC_CTL_OUTSWAP_Msk;
    CRPT->HMAC_DMACNT = 64;
    CRPT->HMAC_CTL |= CRPT_HMAC_CTL_START_Msk;

    /* Start to calculate ... */
    while(bytes > 0)
    {
        if(bytes < 64)
            CRPT->HMAC_DMACNT = (uint32_t)bytes;

        if(CRPT->HMAC_STS & CRPT_HMAC_STS_DATINREQ_Msk)
        {
            data = (int32_t)inpw(addr);
            addr += 4;
            bytes -= 4;

            if(bytes <= 0)
                bytes = 0;

            /* bytes means remain byte counts */
            if(bytes != 0)
            {
                CRPT->HMAC_DATIN = (uint32_t)data;
            }
            else
            {
                /* It's last word ... *-* */
                CRPT->HMAC_CTL |= CRPT_HMAC_CTL_START_Msk | CRPT_HMAC_CTL_DMALAST_Msk;
                CRPT->HMAC_DATIN = (uint32_t)data;
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
                while(CRPT->HMAC_STS & CRPT_HMAC_STS_BUSY_Msk)
                {
                    if(--u32TimeOutCnt == 0)
                        return -1;
                }

                for(i = 0; i < 16; i++)
                    pu32Digest[i] = *(uint32_t *)((uint32_t) & (CRPT->HMAC_DGST[0]) + ((uint32_t)i * 4));
            }
        }
    }

#if (DBG_EN == 1) // enable for debugging
    printf("\nCal_SHA512_Flash\n");
    printf("    0x%08x\n", pu32Digest[0]);
    printf("    0x%08x\n", pu32Digest[1]);
    printf("    0x%08x\n", pu32Digest[2]);
    printf("    0x%08x\n", pu32Digest[3]);
    printf("    0x%08x\n", pu32Digest[4]);
    printf("    0x%08x\n", pu32Digest[5]);
    printf("    0x%08x\n", pu32Digest[6]);
    printf("    0x%08x\n", pu32Digest[7]);
    printf("    0x%08x\n", pu32Digest[8]);
    printf("    0x%08x\n", pu32Digest[9]);
    printf("    0x%08x\n", pu32Digest[10]);
    printf("    0x%08x\n", pu32Digest[11]);
    printf("    0x%08x\n", pu32Digest[12]);
    printf("    0x%08x\n", pu32Digest[13]);
    printf("    0x%08x\n", pu32Digest[14]);
    printf("    0x%08x\n", pu32Digest[15]);
#endif

    return 0;
}

int32_t Cal_SHA512_SRAM(uint32_t u32Addr, uint32_t u32Bytes, uint32_t *pu32Digest)
{
    uint32_t u32TimeOutCnt;

    /* Reset CRYPTO module */
    SYS_ResetModule(CRPT_MODULE);

    /*---------------------------------------
     *  SHA-256
     *---------------------------------------*/
    SHA_Open(CRPT, SHA_MODE_SHA512, SHA_IN_OUT_SWAP, 0);

    SHA_SetDMATransfer(CRPT, u32Addr, u32Bytes);

    SHA_Start(CRPT, CRYPTO_DMA_ONE_SHOT);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(SHA_GET_INT_FLAG(CRPT) == 0)
    {
        if(--u32TimeOutCnt == 0)
            return -1;
    }
    SHA_CLR_INT_FLAG(CRPT);

    SHA_Read(CRPT, pu32Digest);

#if (DBG_EN == 1) // enable for debugging
    printf("\nCal_SHA512_SRAM\n");
    printf("    0x%08x\n", pu32Digest[0]);
    printf("    0x%08x\n", pu32Digest[1]);
    printf("    0x%08x\n", pu32Digest[2]);
    printf("    0x%08x\n", pu32Digest[3]);
    printf("    0x%08x\n", pu32Digest[4]);
    printf("    0x%08x\n", pu32Digest[5]);
    printf("    0x%08x\n", pu32Digest[6]);
    printf("    0x%08x\n", pu32Digest[7]);
    printf("    0x%08x\n", pu32Digest[8]);
    printf("    0x%08x\n", pu32Digest[9]);
    printf("    0x%08x\n", pu32Digest[10]);
    printf("    0x%08x\n", pu32Digest[11]);
    printf("    0x%08x\n", pu32Digest[12]);
    printf("    0x%08x\n", pu32Digest[13]);
    printf("    0x%08x\n", pu32Digest[14]);
    printf("    0x%08x\n", pu32Digest[15]);
#endif

    return 0;
}

int32_t VerifyNuBL3x(uint32_t *pu32FwInfo, uint32_t u32InfoBase)
{
    FW_INFO_T   *pFwInfo;
    uint32_t    u32Start, u32Size, au32Hash[16];
    uint32_t    tmp[17];
    char m[129], Qx[132], Qy[132], R[132], S[132];

    (void)u32InfoBase;

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Enable crypto interrupt */
    NVIC_EnableIRQ(CRPT_IRQn);

    /* Get NuBL3x FW info */
    pFwInfo = (FW_INFO_T *)pu32FwInfo;


    /*---------------------------------------------------------------------------------------------------------*/
    /*  Authenticate NuBL3x FW info ECDSA signature                                                            */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Calculate message (NuBL3x FW info hash) */
    u32Start = (uint32_t)pFwInfo;
    u32Size  = sizeof(FW_INFO_T) - sizeof(ECDSA_SIGN_T);
    if (Cal_SHA512_SRAM(u32Start, u32Size, (uint32_t *)au32Hash) != 0 )
    {
        return -1;
    }
    memcpy((void*)au32Hash, (uint32_t *)au32Hash, sizeof(au32Hash));
    BytesSwap((char*)au32Hash,  sizeof(au32Hash));
    CRPT_Reg2Hex(128, au32Hash, m);

    /* Get Qx */
    memcpy((void*)tmp, (uint32_t *)pFwInfo->pubkey.au32Key0, sizeof(tmp));
    BytesSwap((char*)tmp,  sizeof(tmp));
    CRPT_Reg2Hex(131, tmp, Qx);

    /* Get Qy */
    memcpy((void*)tmp, (uint32_t *)pFwInfo->pubkey.au32Key1, sizeof(tmp));
    BytesSwap((char*)tmp,  sizeof(tmp));
    CRPT_Reg2Hex(131, tmp, Qy);

    /* Get R */
    memcpy((void*)tmp, (uint32_t *)pFwInfo->sign.au32R, sizeof(tmp));
    BytesSwap((char*)tmp,  sizeof(tmp));
    CRPT_Reg2Hex(131, tmp, R);

    /* Get S */
    memcpy((void*)tmp, (uint32_t *)pFwInfo->sign.au32S, sizeof(tmp));
    BytesSwap((char*)tmp,  sizeof(tmp));
    CRPT_Reg2Hex(131, tmp, S);

#if (DBG_EN == 1) // enable for debugging
    printf("\nInput:\n");
    printf(" m\t%s\n", m);
    printf(" Qx\t%s\n", Qx);
    printf(" Qy\t%s\n", Qy);
    printf(" R\t%s\n", R);
    printf(" S\t%s\n", S);
#endif
    ECC_ENABLE_INT(CRPT);
    if(ECC_VerifySignature(CRPT, CURVE_P_521, m, Qx, Qy, R, S) < 0)
    {
#if (DBG_EN == 1) // enable for debugging
        printf("\nVerify ECDSA FAIL !!!\n\n");
#endif
        return -1;
    }


    /*---------------------------------------------------------------------------------------------------------*/
    /*  Verify NuBL3x FW integrity                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Calculate FW hash */
    u32Start = (uint32_t)pFwInfo->mData.au32FwRegion[0].u32Start;
    u32Size  = (uint32_t)pFwInfo->mData.au32FwRegion[0].u32Size;
    if(Cal_SHA512_Flash(u32Start, u32Size, (uint32_t *)au32Hash) != 0)
    {
#if (DBG_EN == 1) // enable for debugging
        printf("\nCalculate FW SHA-512 FAIL !!!\n\n");
#endif
        return -1;
    }
    if(memcmp((void*)&pFwInfo->au32FwHash[0], au32Hash, sizeof(au32Hash)) != 0)
    {
#if (DBG_EN == 1) // enable for debugging
        printf("\nVerify FW SHA-512 FAIL !!!\n\n");
#endif
        return -1;
    }

    return 0;
}
