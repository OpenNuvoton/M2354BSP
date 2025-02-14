/**************************************************************************//**
 * @file     FwInfo.c
 * @version  V1.00
 * @brief    NuBL2 FW Info template and provided by NuBL2 developer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "NuBL2.h"


/*
    Description of FwInfo format:
    [ Reserved ]
        Length:
            Fixed 136 bytes.
        Content:
            It's reserved data block. Not referenced in M2354 secure boot verification.
    [ Metadata data ]
        Length:
            Fixed 4 words + 1 word Extend Info Length(N) + N words Extend Info Content.
        Content:
            Word-0, the ID Hash Configuration.
                bit[1:0], reserved.
                bit[2], indicates whether Info Hash includes PID hash, which will be updated according FwSign.ini after executing FwSign.exe.
                        If supports PID hash, add chip's PID to [PID] section in the FwInfo.ini as shown below example,
                        [PID]
                        PID=0x00235400
                bit[3], indicates whether Info Hash includes UID hash, which will be updated according FwSign.ini after executing FwSign.exe.
                        If supports UID hash, add chip's UID to [UID] section in the FwInfo.ini as shown below example,
                        [UID]
                        UID0=0x11111111
                        UID1=0x22222222
                        UID2=0x33333333
                bit[4], indicates whether Info Hash includes UCID hash, which will be updated according FwSign.ini after executing FwSign.exe.
                        If supports UCID hash, add chip's UCID to [UCID] section in the FwInfo.ini as shown below example,
                        [UCID]
                        UCID0=0xC1111111
                        UCID1=0xC2222222
                        UCID2=0xC3333333
                        UCID3=0xC4444444
                bit[31:5], reserved.
            Word-1, fixed 8(bytes) to indicate only NuBL2 FW region for secure boot verification.
            Word-2, indicates the NuBL2 FW base address.
            Word-3, indicates the NuBL2 FW size, which will be updated after NuBL2 is successfully built.
            Word-4, indicates the valid Extend Info Length. Must be a word alignment length.
            Word-5~, the content of Extend Info.
    [ FW hash ]
        Length:
            Fixed 64 bytes.
        Content:
            To store the NuBL2 FW hash.
            The target content will be updated according FW base and size in Metadata after executing FwSign.exe.
    [ FwInfo signature ]
        Length:
            Fixed 136 bytes.
        Content:
            To store the ECDSA signature.
            The target content will be updated according the ECC private key in FwSign.ini after executing FwSign.exe.
*/
const uint32_t g_InitialFWInfo[] =
{
    /* Reserved - 68x2-bytes (521(544)-bits Qx + 521(544)-bits Qy) */
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,

    /* Metadata data - includes ID Hash Configuration, FW region and Extend Info */
    0x00000001, 0x00000008, 0x00000000, 0x00009420, // Word-2: 0x00000000, NuBL2 FW base
    0x0000000C, 0x20250211, 0x00000000, 0x00000000, // Word-5/6/7: 0x20250211/0x00000000/0x00000000, Extend Info

    /* FW SHA-512, 64-bytes */
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    
    /* FwInfo signature - 68x2-bytes (521(544)-bits R + 521(544)-bits S */ 
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
#if 0
/* Example */
//3c77c81edf5bda3a49caeceae36d6c233cb64694724469b71010e48eb3812ff11fa8834d4285e86edaf9ea576827b25065f6db4f953bf76b9a7c3099f329b02f
    /* FW SHA-512, 64-bytes */
    0x1ec8773c, 0x3ada5bdf, 0xeaecca49, 0x236c6de3,
    0x9446b63c, 0xb7694472, 0x8ee41010, 0xf12f81b3,
    0x4d83a81f, 0x6ee88542, 0x57eaf9da, 0x50b22768,
    0x4fdbf665, 0x6bf73b95, 0x99307c9a, 0x2fb029f3,
    
    /* FwInfo signature - 68x2-bytes (521(544)-bits R + 521(544)-bits S */ 
//07225b1ae9106017b22b6272425da09079ddacfb57defe40ed6742c0d06dc1ee3eb67bae765e3b0b0470c6ce41494cacbf8fd774c259ec1c23138d1c6c73539bf37
    0x72000000, 0x91aeb125, 0x227b0106, 0x252427b6,
    0x9d0709da, 0x7db5cfda, 0xd60ee4ef, 0x060d2c74,
    0xebe31edc, 0x65e7ba67, 0x47b0b0e3, 0x14e46c0c,
    0xf8cbca94, 0x254c77fd, 0x31c2c19e, 0xc7c6d138, 0x37bf3935, 
//1f53faaed1daa9823324404df7be882c5e63888225a99afd6c9611b6d6c3c8a8f27bc5a7b61819d2bc7103c2b15668b6bf81f452410d4727301d1170b77ad94fab4    
    0xf5010000, 0x1dedaa3f, 0x322398aa, 0x7bdf0444,
    0xe6c582e8, 0x5a228838, 0xc9d6af99, 0x6c6d1b61,
    0x278f8a3c, 0x617b5abc, 0xc72b9d81, 0x152b3c10,
    0xf86b8b66, 0x1024451f, 0x017372d4, 0x770b17d1, 0xb4fa94ad,
#endif    
};

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
