/**************************************************************************//**
 * @file     main.c
 * @version  V1.10
 * @brief    Demonstrate how to encrypt/decrypt data by AES GCM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define _SWAP

#define MAX_GCM_BUF     4096
__ALIGNED(4) uint8_t g_au8Buf[MAX_GCM_BUF];
__ALIGNED(4) uint8_t g_au8Out[MAX_GCM_BUF];
__ALIGNED(4) uint8_t g_au8Out2[MAX_GCM_BUF];

/* for the key and data in binary format */
__ALIGNED(4) uint8_t g_key[32] = { 0 };
__ALIGNED(4) uint8_t g_iv[32] = { 0 };
__ALIGNED(4) uint8_t g_A[265] = { 0 };
__ALIGNED(4) uint8_t g_P[256] = { 0 };
__ALIGNED(4) uint8_t g_C[256] = { 0 };
__ALIGNED(4) uint8_t g_T[256] = { 0 };


typedef struct {
    char *pchKey;   /* The block cipher key */
    char *pchIV;    /* The initialization vector */
    char *pchA;     /* The additional authenticated data */
    char *pchP;     /* The plaintext */
    char *pchC;     /* The ciphertext */
    char *pchTag;   /* The authentication tag */
} GCM_TEST_T;

/* Test items */
const GCM_TEST_T sElements[] ={
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D58503B9699DE785895A96FDBAAF43B1CD7F598ECE23881B00E3ED0306887B0C785E27E8AD3F8223207104725DD4",
        "",
        "",
        "5F91D77123EF5EB9997913849B8DC1E9"
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "",
        "",
        "3247184B3C4F69A44DBCD22887BBB418",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B391AAFD255",
        "42831EC2217774244B7221B784D0D49CE3AA212F2C02A4E035C17E2329ACA12E21D514B25466931C7D8F6A5AAC84AA051BA30B396A0AAC973D58E091473F5985",
        "4D5C2AF327CD64A62CF35ABD2BA6FAB4",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D58503B9699DE785895A96FDBAAF43B1CD7F598ECE23881B00E3ED0306887B0C785E27E8AD3F8223207104725DD4",
        "",
        "",
        "5F91D77123EF5EB9997913849B8DC1E9",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D58503B9699DE785895A96FDBAAF43B1CD7F598ECE23881B00E3ED0306887B0C785E27E8AD3F8223207104725DD4",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B391AAFD255",
        "42831EC2217774244B7221B784D0D49CE3AA212F2C02A4E035C17E2329ACA12E21D514B25466931C7D8F6A5AAC84AA051BA30B396A0AAC973D58E091473F5985",
        "64C0232904AF398A5B67C10B53A5024D",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "42831EC2217774244B7221B784D0D49CE3AA212F2C02A4E035C17E2329ACA12E21D514B25466931C7D8F6A5AAC84AA051BA30B396A0AAC973D58E091",
        "F07C2528EEA2FCA1211F905E1B6A881B",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "42831EC2217774244B7221B784D0D49CE3AA212F2C02A4E035C17E2329ACA12E21D514B25466931C7D8F6A5AAC84AA051BA30B396A0AAC973D58E091",
        "F07C2528EEA2FCA1211F905E",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "",
        "",
        "C835AA88AEBBC94F5A02E179FDCFC3E4",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B391AAFD255",
        "3980CA0B3C00E841EB06FAC4872A2757859E1CEAA6EFD984628593B40CA1E19C7D773D00C144C525AC619D18C84A3F4718E2448B2FE324D9CCDA2710ACADE256",
        "9924A7C8587336BFB118024DB8674A14",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D58503B9699DE785895A96FDBAAF43B1CD7F598ECE23881B00E3ED0306887B0C785E27E8AD3F8223207104725DD4",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B391AAFD255",
        "3980CA0B3C00E841EB06FAC4872A2757859E1CEAA6EFD984628593B40CA1E19C7D773D00C144C525AC619D18C84A3F4718E2448B2FE324D9CCDA2710ACADE256",
        "3B9153B4E7318A5F3BBEAC108F8A8EDB",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "3980CA0B3C00E841EB06FAC4872A2757859E1CEAA6EFD984628593B40CA1E19C7D773D00C144C525AC619D18C84A3F4718E2448B2FE324D9CCDA2710",
        "93EA28C659E269902A80ACD208E7FC80",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "3980CA0B3C00E841EB06FAC4872A2757859E1CEAA6EFD984628593B40CA1E19C7D773D00C144C525AC619D18C84A3F4718E2448B2FE324D9CCDA2710",
        "93EA28C659E269902A80ACD2",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "",
        "",
        "FD2CAA16A5832E76AA132C1453EEDA7E",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B391AAFD255",
        "522DC1F099567D07F47F37A32A84427D643A8CDCBFE5C0C97598A2BD2555D1AA8CB08E48590DBB3DA7B08B1056828838C5F61E6393BA7A0ABCC9F662898015AD",
        "B094DAC5D93471BDEC1A502270E3CC6C",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D58503B9699DE785895A96FDBAAF43B1CD7F598ECE23881B00E3ED0306887B0C785E27E8AD3F8223207104725DD4",
        "",
        "",
        "DE34B6DCD4CEE2FDBEC3CEA01AF1EE44",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "522DC1F099567D07F47F37A32A84427D643A8CDCBFE5C0C97598A2BD2555D1AA8CB08E48590DBB3DA7B08B1056828838C5F61E6393BA7A0ABCC9F662",
        "E097195F4532DA895FB917A5A55C6AA0",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "522DC1F099567D07F47F37A32A84427D643A8CDCBFE5C0C97598A2BD2555D1AA8CB08E48590DBB3DA7B08B1056828838C5F61E6393BA7A0ABCC9F662",
        "E097195F4532DA895FB917A5",
    },

    {
        "f8d6868a7250f76e85de2e9f813edfc2",
        "0a",
        "30307aef4c3b7fa25ac7b181999851717f703a481bf59b16546bf2df7fc7d81677de6989cc64140470ab8b86a42ae498",
        "81e562083769c8ae8dfda00f192396a504b70dcea2c25ed0b89012ab9ebffbdad8f227d98951e75685b16bac064ceebd6b1840",
        "d1c2fa6ba5b29cb95f7819b2e6f2a7dbc0d8a58828f7e8528451633385afe0730921d08b50b7e0fa3be469cc72ff0e3226fb54",
        "b96cb72d696ad2325c36a55634a21d0f"
    },
};

void DumpBuffHex(uint8_t *pucBuff, int nBytes)
{
    int32_t i32Idx, i, len;


    i32Idx = 0;
    while(nBytes > 0)
    {
        printf("0x%04X  ", i32Idx);

        len = (nBytes < 16)?nBytes:16;
        for(i = 0; i < len; i++)
            printf("%02x ", pucBuff[i32Idx + i]);
        for(; i < 16; i++)
        {
            printf("   ");
        }
        printf("  ");
        for(i = 0; i < len; i++)
        {
            if((pucBuff[i32Idx + i] >= 0x20) && (pucBuff[i32Idx + i] < 127))
                printf("%c", pucBuff[i32Idx + i]);
            else
                printf(".");
            nBytes--;
        }
        i32Idx += len;
        printf("\n");
    }
    printf("\n");
}


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

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA6_Msk | UART0_TXD_PA7_Msk))) | UART0_RXD_PA6 | UART0_TXD_PA7;

}



void UART0_Init(void)
{

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


volatile int  g_Crypto_Int_done = 0;

void CRPT_IRQHandler()
{
    if(AES_GET_INT_FLAG(CRPT))
    {
        g_Crypto_Int_done = 1;
        AES_CLR_INT_FLAG(CRPT);
    }
}


void str2bin(const char *pstr, uint8_t *buf, uint32_t size)
{
    uint32_t i;
    uint8_t u8Ch;
    char c;

    for(i = 0; i < size; i++)
    {
        c = *pstr++;
        if(c == NULL)
            break;

        if((c >= 'a') && (c <= 'f'))
            c -= ('a' - 10);
        else if((c >= 'A') && (c <= 'F'))
            c -= ('A' - 10);
        else if((c >= '0') && (c <= '9'))
            c -= '0';
        u8Ch = (uint8_t)c << 4;

        c = *pstr++;
        if(c == NULL)
        {
            buf[i] = u8Ch;
            break;
        }

        if((c >= 'a') && (c <= 'f'))
            c -= ('a' - 10);
        else if((c >= 'A') && (c <= 'F'))
            c -= ('A' - 10);
        else if((c >= '0') && (c <= '9'))
            c -= '0';
        u8Ch += (uint8_t)c;

        buf[i] = u8Ch;
    }

}


void bin2str(uint8_t *buf, uint32_t size, char *pstr)
{
    int32_t i;
    uint8_t c;
    
    for(i=size-1;i >= 0;i--)
    {
        c = buf[i] >> 4;
        *pstr++ = (c >= 10)?c-10+'a':c+'0';
        c = buf[i] & 0xf;
        *pstr++ = (c >= 10)?c-10+'a':c+'0';
    }
    
    *pstr = '\0';
}



int32_t ToBigEndian(uint8_t *pbuf, uint32_t u32Size)
{
    uint32_t i;
    uint8_t u8Tmp;
    uint32_t u32Tmp;

    /* pbuf must be word alignment */
    if((uint32_t)pbuf & 0x3)
    {
        printf("The buffer must be 32-bit alignment.");
        return -1;
    }

    while(u32Size >= 4)
    {
        u8Tmp = *pbuf;
        *(pbuf) = *(pbuf + 3);
        *(pbuf + 3) = u8Tmp;

        u8Tmp = *(pbuf + 1);
        *(pbuf + 1) = *(pbuf + 2);
        *(pbuf + 2) = u8Tmp;

        u32Size -= 4;
        pbuf += 4;
    }

    if(u32Size > 0)
    {
        u32Tmp = 0;
        for(i = 0; i < u32Size; i++)
        {
            u32Tmp |= *(pbuf + i) << (24 - i * 8);
        }

        *((uint32_t *)pbuf) = u32Tmp;
    }
    
    return 0;
}



int32_t ToLittleEndian(uint8_t *pbuf, uint32_t u32Size)
{
    uint32_t i;
    uint8_t u8Tmp;
    uint32_t u32Tmp;

    /* pbuf must be word alignment */
    if((uint32_t)pbuf & 0x3)
    {
        printf("The buffer must be 32-bit alignment.");
        return -1;
    }

    while(u32Size >= 4)
    {
        u8Tmp = *pbuf;
        *(pbuf) = *(pbuf + 3);
        *(pbuf + 3) = u8Tmp;

        u8Tmp = *(pbuf + 1);
        *(pbuf + 1) = *(pbuf + 2);
        *(pbuf + 2) = u8Tmp;

        u32Size -= 4;
        pbuf += 4;
    }

    if(u32Size > 0)
    {
        u32Tmp = 0;
        for(i = 0; i < u32Size; i++)
        {
            u32Tmp |= *(pbuf + i) << (24 - i * 8);
        }

        *((uint32_t *)pbuf) = u32Tmp;
    }

    return 0;
}



/*
    GCM input format must be block alignment. The block size is 16 bytes.
    {IV}{IV nbits}{A}{P/C}

*/

int32_t AES_GCMPacker(uint8_t *iv, uint32_t iv_len, uint8_t *A, uint32_t A_len, uint8_t *P, uint32_t P_len, uint8_t *pbuf, uint32_t *psize)
{
    uint32_t i;
    uint32_t iv_len_aligned, A_len_aligned, P_len_aligned;
    uint32_t u32Offset = 0;

    if(iv_len > 0)
    {
        iv_len_aligned = iv_len;
        if(iv_len & 0xful)
            iv_len_aligned = ((iv_len + 16) / 16) * 16;

        /* fill iv to output */
        for(i = 0; i < iv_len_aligned; i++)
        {
            if(i < iv_len)
                pbuf[i] = iv[i];
            else
                pbuf[i] = 0; // padding zero
        }

#ifndef _SWAP
        ToBigEndian(pbuf, iv_len);
#endif

        /* fill iv len to putput */
        if(iv_len == 12)
        {
#ifdef _SWAP
            pbuf[15] = 1;
#else
            pbuf[12] = 1;
#endif
            u32Offset = iv_len_aligned;
        }
        else
        {
            /* Note: IV could be 2^64 but we don't expected it is so large here */
            for(i = iv_len_aligned; i < iv_len_aligned + 8; i++)
            {
                pbuf[i] = 0;
            }

            *((uint32_t *)&pbuf[iv_len_aligned + 8]) = 0; // high word
            *((uint32_t*)&pbuf[iv_len_aligned + 12]) = iv_len * 8; // low word
#ifdef _SWAP
            ToBigEndian(&pbuf[iv_len_aligned + 12], 4);
#endif

            u32Offset = iv_len_aligned + 16;
        }
    }


    /* Fill A */
    if(A_len > 0)
    {
        A_len_aligned = A_len;
        if(A_len & 0xful)
            A_len_aligned = ((A_len + 16) / 16) * 16;

        for(i = 0; i < A_len_aligned; i++)
        {
            if(i < A_len)
                pbuf[u32Offset + i] = A[i];
            else
                pbuf[u32Offset + i] = 0; // padding zero
        }
        
#ifndef _SWAP
        ToBigEndian(&pbuf[u32Offset], A_len);
#endif

        u32Offset += A_len_aligned;
    }

    /* Fill P/C */
    if(P_len > 0)
    {
        P_len_aligned = P_len;
        if(P_len & 0xful)
            P_len_aligned = ((P_len + 16) / 16) * 16;

        for(i = 0; i < P_len_aligned; i++)
        {
            if(i < P_len)
                pbuf[u32Offset + i] = P[i];
            else
                pbuf[u32Offset + i] = 0; // padding zero
        }

#ifndef _SWAP
        ToBigEndian(&pbuf[u32Offset], P_len);
#endif

        u32Offset += P_len_aligned;
    }

    *psize = u32Offset;

    return 0;
}


int32_t AES_GCMEnc(uint8_t *key, uint32_t klen, uint8_t *iv, uint32_t ivlen, uint8_t *A, uint32_t alen, uint8_t *P, uint32_t plen, uint8_t *buf, uint32_t *size, uint32_t *plen_aligned)
{
    __ALIGNED(4) uint8_t au8TmpBuf[32] = { 0 };

    printf("\n");

    printf("key (%d):\n", klen);
    DumpBuffHex(key, klen);

    printf("IV (%d):\n",ivlen);
    DumpBuffHex(iv, ivlen);

    printf("A (%d):\n", alen);
    DumpBuffHex(A, alen);

    printf("P (%d):\n", plen);
    DumpBuffHex(P, plen);

    /* Prepare the blocked buffer for GCM */
    AES_GCMPacker(iv, ivlen, A, alen, P, plen, g_au8Buf, size);
    *plen_aligned = (plen & 0xful) ? (plen + 16) / 16 * 16 : plen;


    printf("input blocks (%d):\n", *size);
    DumpBuffHex(g_au8Buf, *size);


    memcpy(au8TmpBuf, key, klen);
    ToBigEndian(au8TmpBuf, klen);

    if(klen == 16)
    {
        AES_Open(CRPT, 0, 1, AES_MODE_GCM, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
        AES_SetKey(CRPT, 0, (uint32_t *)au8TmpBuf, AES_KEY_SIZE_128);
    }
    else if(klen == 24)
    {
        AES_Open(CRPT, 0, 1, AES_MODE_GCM, AES_KEY_SIZE_192, AES_IN_OUT_SWAP);
        AES_SetKey(CRPT, 0, (uint32_t *)au8TmpBuf, AES_KEY_SIZE_192);
    }
    else
    {
        printf("key size 256\n");
        AES_Open(CRPT, 0, 1, AES_MODE_GCM, AES_KEY_SIZE_256, AES_IN_OUT_SWAP);
        AES_SetKey(CRPT, 0, (uint32_t *)au8TmpBuf, AES_KEY_SIZE_256);
    }

    /* Set byte count of IV */
    CRPT->AES_GCM_IVCNT[0] = ivlen;
    CRPT->AES_GCM_IVCNT[1] = 0;

    /* Set bytes count of A */
    CRPT->AES_GCM_ACNT[0] = alen;
    CRPT->AES_GCM_ACNT[1] = 0;
    CRPT->AES_GCM_PCNT[0] = plen;
    CRPT->AES_GCM_PCNT[1] = 0;


    AES_SetDMATransfer(CRPT, 0, (uint32_t)g_au8Buf, (uint32_t)buf, *size);

    g_Crypto_Int_done = 0;
    /* Start AES Eecrypt */
    AES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    /* Waiting for AES calculation */
    while(!g_Crypto_Int_done);


    printf("output blocks (%d):\n", *size);
    DumpBuffHex(buf, *size);

    return 0;
}

int32_t AES_GCMDec(uint8_t *key, uint32_t klen, uint8_t *iv, uint32_t ivlen, uint8_t *A, uint32_t alen, uint8_t *P, uint32_t plen, uint8_t *buf, uint32_t *size, uint32_t *plen_aligned)
{
    __ALIGNED(4) uint8_t au8Buf[32]; /* buffer for input packer */

    printf("\n");

    printf("key (%d):\n", klen);
    DumpBuffHex(key, klen);

    printf("IV (%d):\n", ivlen);
    DumpBuffHex(iv, ivlen);

    printf("A (%d):\n", alen);
    DumpBuffHex(A, alen);

    /* Prepare the blocked buffer for GCM */
    AES_GCMPacker(iv, ivlen, A, alen, P, plen, g_au8Buf, size);
    *plen_aligned = (plen & 0xful) ? (plen + 16) / 16 * 16 : plen;

    printf("input blocks (%d):\n", *size);
    DumpBuffHex(g_au8Buf, *size);

    memcpy(au8Buf, key, klen);
    ToBigEndian(au8Buf, klen);

    if(klen == 16)
    {
        AES_Open(CRPT, 0, 0, AES_MODE_GCM, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
        AES_SetKey(CRPT, 0, (uint32_t *)au8Buf, AES_KEY_SIZE_128);
    }
    else if(klen == 24)
    {
        AES_Open(CRPT, 0, 0, AES_MODE_GCM, AES_KEY_SIZE_192, AES_IN_OUT_SWAP);
        AES_SetKey(CRPT, 0, (uint32_t *)au8Buf, AES_KEY_SIZE_192);
    }
    else
    {
        AES_Open(CRPT, 0, 0, AES_MODE_GCM, AES_KEY_SIZE_256, AES_IN_OUT_SWAP);
        AES_SetKey(CRPT, 0, (uint32_t *)au8Buf, AES_KEY_SIZE_256);
    }

    /* Set byte count of IV */
    CRPT->AES_GCM_IVCNT[0] = ivlen;
    CRPT->AES_GCM_IVCNT[1] = 0;

    /* Set bytes count of A */
    CRPT->AES_GCM_ACNT[0] = alen;
    CRPT->AES_GCM_ACNT[1] = 0;
    CRPT->AES_GCM_PCNT[0] = plen;
    CRPT->AES_GCM_PCNT[1] = 0;


    AES_SetDMATransfer(CRPT, 0, (uint32_t)g_au8Buf, (uint32_t)buf, *size);

    g_Crypto_Int_done = 0;
    /* Start AES Eecrypt */
    AES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    /* Waiting for AES calculation */
    while(!g_Crypto_Int_done);

    printf("output blocks (%d):\n", *size);
    DumpBuffHex(buf, *size);

    return 0;
}


/*-----------------------------------------------------------------------------*/
int main(void)
{
    int i,n;
    uint32_t size, klen, plen, tlen,plen_aligned, alen, ivlen;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); 

    /* Initialize UART0 */
    UART0_Init();                      

    printf("+---------------------------------------+\n");
    printf("|               AES GCM Test            |\n");
    printf("+---------------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    
    n = sizeof(sElements)/sizeof(GCM_TEST_T);
    for(i=0;i<n;i++)
    {
        
        printf("\n============================================================================\n");
        printf("Test iter %d/%d\n\n", i+1, n);
        
        
        SYS->IPRST0 |= SYS_IPRST0_CRPTRST_Msk;
        SYS->IPRST0 ^= SYS_IPRST0_CRPTRST_Msk;
        
        AES_ENABLE_INT(CRPT);
        
        printf("key = %s\n", sElements[i].pchKey);
        printf("iv  = %s\n", sElements[i].pchIV);
        printf("A   = %s\n", sElements[i].pchA);
        printf("P   = %s\n", sElements[i].pchP);

        klen = strlen(sElements[i].pchKey) / 2;
        ivlen = strlen(sElements[i].pchIV) / 2;
        alen = strlen(sElements[i].pchA) / 2;
        plen = strlen(sElements[i].pchP) / 2;
        tlen = strlen(sElements[i].pchTag) / 2;

        str2bin(sElements[i].pchKey, g_key, klen);
        str2bin(sElements[i].pchIV, g_iv, ivlen);
        str2bin(sElements[i].pchA, g_A, alen);
        str2bin(sElements[i].pchP, g_P, plen);

        AES_GCMEnc(g_key, klen, g_iv, ivlen, g_A, alen, g_P, plen, g_au8Out, &size, &plen_aligned);

#ifndef _SWAP
        ToLittleEndian(g_au8Out, size);
#endif

        printf("C=%s\n", sElements[i].pchC);
        printf("T=%s\n", sElements[i].pchTag);

        str2bin(sElements[i].pchC, g_C, plen);
        str2bin(sElements[i].pchTag, g_T, tlen);

        if(memcmp(g_C, g_au8Out, plen))
        {
            printf("ERR: Encrypted data fail!\n");
            while(1){}
        }
        
        if(memcmp(g_T, &g_au8Out[plen_aligned], tlen))
        {
            printf("ERR: Tag fail!");
            while(1) {}
        }

        AES_GCMDec(g_key, klen, g_iv, ivlen, g_A, alen, g_au8Out, plen, g_au8Out2, &size, &plen_aligned);


        if(memcmp(g_P, g_au8Out2, plen))
        {
            printf("ERR: Encrypted data fail!\n");
            while(1) {}
        }

        if(memcmp(g_T, &g_au8Out2[plen_aligned], tlen))
        {
            printf("ERR: Tag fail!");
            while(1) {}
        }

        printf("Test PASS!\n");
    
    }

    while(1){}
        
}
