/**************************************************************************//**
 * @file     lcdlib.c
 * @version  V3.00
 * @brief    RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD library source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

#include "lcdlib.h"

/** @addtogroup Library Library
  @{
*/

/** @addtogroup LCDLIB LCD Library
  @{
*/

/** @addtogroup LCDLIB_EXPORTED_FUNCTIONS LCD Library Exported Functions
  @{
*/

/**
 *  @brief Display text on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  InputStr    Text string to show on display
 *
 *  @return None
 */
void LCDLIB_Printf(uint32_t u32Zone, char *InputStr)
{
    uint32_t    i, index, ch, len;
    uint16_t    DispData;
    uint32_t    com, seg;

    len = strlen(InputStr);

    /* Fill out all characters on display */
    for(index = 0; index < g_LCDZoneInfo[u32Zone].u8LCDDispTableNum; index++)
    {
        if(index < len)
        {
            ch = *InputStr;
        }
        else
        {
            /* Padding with SPACE */
            ch = 0x20;
        }

        if(u32Zone == ZONE_MAIN_DIGIT)
        {
            /* The Main Digit Table is an ASCII table beginning with "SPACE" (hex is 0x20) */
            ch       = ch - 0x20;
            DispData = *(g_LCDZoneInfo[u32Zone].pu16LCDDispTable + ch);
        }
        /* For Other Zones (Support '0' ~ '9' only) */
        else if((ch >= '0') && (ch <= '9'))
        {
            ch = ch - '0';
            DispData = *(g_LCDZoneInfo[u32Zone].pu16LCDDispTable + ch);
        }
        /* Out of definition. Will show "SPACE" */
        else
        {
            DispData = 0;
        }

        for(i = 0; i < g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            com = *(g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg
                    + (index * g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2)
                    + (i * 2) + 0);
            seg = *(g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg
                    + (index * g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2)
                    + (i * 2) + 1);

            if(DispData & (1 << i))
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0);
            }
        }
        
        InputStr++;
    }
}

/**
 *  @brief Display unsigned number on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  InputNum    unsigned number to show on display
 *
 *  @return None
 */
void LCDLIB_PrintNumber(uint32_t u32Zone, uint32_t InputNum)
{
    uint32_t    i, index, val, div;
    uint16_t    DispData;
    uint32_t    com, seg;

    /* Extract useful digits */
    div = 1;

    /* Fill out all characters on display */
    index = g_LCDZoneInfo[u32Zone].u8LCDDispTableNum;
    while(index != 0)
    {
        index--;

        val = (InputNum / div) % 10;
        if(u32Zone == ZONE_MAIN_DIGIT)
            val += 16; /* The Main Digit Table is an ASCII table beginning with "SPACE" */

        DispData = *(g_LCDZoneInfo[u32Zone].pu16LCDDispTable + val);

        for(i = 0; i < g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            com = *(g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg
                    + (index * g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2)
                    + (i * 2) + 0);
            seg = *(g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg
                    + (index * g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2)
                    + (i * 2) + 1);

            if(DispData & (1 << i))
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0);
            }
        }

        div = div * 10;
    }
}

/**
 *  @brief Display signed number on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  iInputNum   signed number to show on display
 *  @param[in]  u8DigiCnt   valid digital number count
 *
 *  @return None
 */
void LCDLIB_PrintNumberEx(uint32_t u32Zone, int32_t iInputNum, uint8_t u8DigiCnt)
{
    uint32_t    i, index, val, div;
    uint16_t    DispData;
    uint32_t    com, seg;
    uint8_t     is_negative = 0;

    /* Extract useful digits */
    div = 1;

    if(iInputNum < 0)
    {
        is_negative = 1;
        iInputNum = 0 - iInputNum;
    }

    /* Fill out all characters on display */
    index = g_LCDZoneInfo[u32Zone].u8LCDDispTableNum;
    while(index != 0)
    {
        index--;

        if(u8DigiCnt == 0)
            continue;
        u8DigiCnt --;

        val = ((uint32_t)iInputNum / div) % 10;
        if(u32Zone == ZONE_MAIN_DIGIT)
            val += 16; /* The Main Digit Table is an ASCII table beginning with "SPACE" */

        DispData = *(g_LCDZoneInfo[u32Zone].pu16LCDDispTable + val);

        for(i = 0; i < g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            com = *(g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg
                    + (index * g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2)
                    + (i * 2) + 0);
            seg = *(g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg
                    + (index * g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2)
                    + (i * 2) + 1);

            if (DispData & (1 << i))
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0);
            }
        }

        div = div * 10;
    }

    if(u32Zone == ZONE_MAIN_DIGIT)
        LCDLIB_SetSymbol(SYMBOL_MINUS_2, is_negative);
}

/**
 *  @brief Display character on LCD
 *
 *  @param[in]  u32Zone     the assigned number of display area
 *  @param[in]  u32Index    the requested display position in zone
 *  @param[in]  u8Ch        Character to show on display
 *
 *  @return None
 */
void LCDLIB_PutChar(uint32_t u32Zone, uint32_t u32Index, uint8_t u8Ch)
{
    uint32_t    i, ch;
    uint16_t    DispData;
    uint32_t    com, seg;

    if(u32Index <= g_LCDZoneInfo[u32Zone].u8LCDDispTableNum)
    {
        /* For Main Zone */
        if(u32Zone == ZONE_MAIN_DIGIT)
        {
            /* Defined letters currently starts at "SPACE" - 0x20; */
            ch       = u8Ch - 0x20;
            DispData = *(g_LCDZoneInfo[u32Zone].pu16LCDDispTable + ch);
        }
        /* For Other Zones (Support '0' ~ '9' only) */
        else if((u8Ch >= '0') && (u8Ch <= '9'))
        {
            u8Ch = u8Ch - '0';
            DispData = *(g_LCDZoneInfo[u32Zone].pu16LCDDispTable + u8Ch);
        }
        /* Out of definition. Will show "SPACE" */
        else
        {
            DispData = 0;
        }

        for(i = 0; i < g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum; i++)
        {
            com = *(g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg
                    + (u32Index * g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2)
                    + (i * 2) + 0);

            seg = *(g_LCDZoneInfo[u32Zone].pu8GetLCDComSeg
                    + (u32Index * g_LCDZoneInfo[u32Zone].u8GetLCDComSegNum * 2)
                    + (i * 2) + 1);

            if(DispData & (1 << i))
            {
                /* Turn on display */
                LCD_SetPixel(com, seg, 1);
            }
            else
            {
                /* Turn off display */
                LCD_SetPixel(com, seg, 0);
            }
        }
    }
}

/**
 *  @brief Display symbol on LCD
 *
 *  @param[in]  u32Symbol   the combination of com, seg position
 *  @param[in]  u32OnOff    1: display symbol
 *                          0: not display symbol
 *
 *  @return     None
 */
void LCDLIB_SetSymbol(uint32_t u32Symbol, uint32_t u32OnOff)
{
    uint32_t com, seg;

    com = (u32Symbol & 0xF);
    seg = ((u32Symbol & 0xFF0) >> 4);

    if(u32OnOff)
        LCD_SetPixel(com, seg, 1); /* Turn on display */
    else
        LCD_SetPixel(com, seg, 0); /* Turn off display */
}

/*@}*/ /* end of group LCDLIB_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group LCDLIB */

/*@}*/ /* end of group Library */
