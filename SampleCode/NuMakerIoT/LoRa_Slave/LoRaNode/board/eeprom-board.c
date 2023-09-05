/****************************************************************************
 * @file     lpm-board.c
 * @version  V1.10
 * @brief    Target board EEPROM driver implementation. Using M252 APROM instead of eeprom.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "NuMicro.h"
#include "utilities.h"
#include "eeprom-board.h"
#include "board-config.h"

static  uint32_t s_au32SectorBuf[FMC_FLASH_PAGE_SIZE / 4];

static void ProgramPage(uint32_t u32StartAddr, uint32_t *pu32Buf)
{
    uint32_t i;

    for(i = 0; i < FMC_FLASH_PAGE_SIZE / 4; i++)
    {
        FMC_Write(u32StartAddr + (i * 4), pu32Buf[i]);
    }
}

static void ReadPage(uint32_t u32StartAddr, uint32_t *pu32Buf)
{
    uint32_t i;

    for(i = 0; i < FMC_FLASH_PAGE_SIZE / 4; i++)
        pu32Buf[i] = FMC_Read(u32StartAddr + (i * 4));
}

static int32_t 
ProgramFlash(
	uint32_t u32FlashAddr,
	uint8_t *pu8Data,
	uint32_t u32DataLen
)
{
	int32_t i32WriteLen;
	int32_t i32EachWriteLen;

	uint32_t u32PageAlignAddr;
	uint32_t u32Offset;
	int32_t i;

	i32WriteLen = u32DataLen;
	
	if((i32WriteLen == FMC_FLASH_PAGE_SIZE) && (( u32FlashAddr & (FMC_FLASH_PAGE_SIZE - 1)) == 0) ){
        /* Page erase */
        FMC_Erase(u32FlashAddr);

        while(i32WriteLen >= FMC_FLASH_PAGE_SIZE)
        {
            ProgramPage(u32FlashAddr, (uint32_t *) pu8Data);
            i32WriteLen -= FMC_FLASH_PAGE_SIZE;
            pu8Data += FMC_FLASH_PAGE_SIZE;
            u32FlashAddr   += FMC_FLASH_PAGE_SIZE;
        }
	}
	else {
		do{
			u32PageAlignAddr = u32FlashAddr & ~(FMC_FLASH_PAGE_SIZE - 1) ; 	//align to FMC_FLASH_PAGE_SIZE(512) address

			/* Get the page offset*/
			u32Offset = (u32FlashAddr & (FMC_FLASH_PAGE_SIZE - 1));

			if(u32Offset || (i32WriteLen < FMC_FLASH_PAGE_SIZE))
			{
				/* Not 512-byte alignment. Read the destination page for modification. Note: It needs to avoid adding MASS_STORAGE_OFFSET twice. */
				ReadPage(u32PageAlignAddr, &s_au32SectorBuf[0]);
			}

			i32EachWriteLen = FMC_FLASH_PAGE_SIZE - u32Offset;
			if(i32WriteLen < i32EachWriteLen)
				i32EachWriteLen = i32WriteLen;

            /* Update the destination buffer */
			uint8_t *pu8TempAddr = ((uint8_t *)s_au32SectorBuf) + u32Offset;

            for(i = 0; i < i32EachWriteLen ; i++)
            {
                pu8TempAddr[i] = pu8Data[i];
            }			
			
            /* Page erase */
            FMC_Erase(u32PageAlignAddr);
            /* Write to the destination page */
            ProgramPage(u32PageAlignAddr, (uint32_t *) s_au32SectorBuf);

			i32WriteLen -= i32EachWriteLen;
			u32FlashAddr += i32EachWriteLen;
			pu8Data += i32EachWriteLen;
			
		}while(i32WriteLen > 0);
	}

	return u32DataLen;
}

uint8_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	if(addr < (DATA_EEPROM_END - DATA_EEPROM_BASE))
		assert_param( addr + DATA_EEPROM_BASE + size <= DATA_EEPROM_END );
	else
		assert_param( addr + DATA_EEPROM_BASE + size <= FMC_ADDR_END );
		
    assert_param( buffer != NULL );

	uint32_t u32FlashAddr;
	uint8_t au8TempBuf[4];
	uint32_t u32AlignBytes;
	uint32_t u32ReadLen = 0;;

    /* Unlock protected registers */
    SYS_UnlockReg();

	/* Enable FMC function for eeprom function */
	FMC_Open();
	
	u32FlashAddr = addr + DATA_EEPROM_BASE;
	
	/*Read first align/unalign data (1 ~ 4 bytes)*/
	u32AlignBytes = (u32FlashAddr & (4 - 1));

	u32FlashAddr = u32FlashAddr - u32AlignBytes;
		
	u32ReadLen = 4 - u32AlignBytes;
	if(u32ReadLen > size)
		u32ReadLen = size;

	*((uint32_t *)au8TempBuf) = FMC_Read(u32FlashAddr);
	memcpy(buffer, au8TempBuf + u32AlignBytes, u32ReadLen);
	u32FlashAddr += 4;
	buffer += u32ReadLen;
	
	/*Read align data (4 * n bytes)*/
	while((size - u32ReadLen) >= 4)
	{
		*((uint32_t *)au8TempBuf) = FMC_Read(u32FlashAddr);
		memcpy(buffer, au8TempBuf, 4);
		u32FlashAddr += 4;
		buffer += 4;
		u32ReadLen += 4;
	}

	/*Read remain data ( < 4 bytes)*/
	if(size != u32ReadLen){
		*((uint32_t *)au8TempBuf) = FMC_Read(u32FlashAddr);
		memcpy(buffer, au8TempBuf, size - u32ReadLen);		
	}

	FMC_Close();

    /* Unlock protected registers */
    SYS_LockReg();
	
	return SUCCESS;
}


uint8_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	if(addr < (DATA_EEPROM_END - DATA_EEPROM_BASE))
		assert_param( addr + DATA_EEPROM_BASE + size <= DATA_EEPROM_END );
	else
		assert_param( addr + DATA_EEPROM_BASE + size <= FMC_ADDR_END );

	if(buffer == NULL){
		return SUCCESS;
	}
	
    /* Unlock protected registers */
    SYS_UnlockReg();

	/* Enable FMC function for eeprom function */
	FMC_Open();
	
    FMC_EnableAPUpdate();
	ProgramFlash(addr + DATA_EEPROM_BASE, buffer, size);
    FMC_DisableAPUpdate();
	
	FMC_Close();

    /* Unlock protected registers */
    SYS_LockReg();

#if 0	
	
	//verify program test code
	static uint8_t s_au8VerifyBuf[1024];
	uint8_t *puReadBuff = s_au8VerifyBuf;
	
	if(puReadBuff == NULL){
		printf("DDDDDDDDDD puReadBuff is NULL \n");
		return SUCCESS;
	}

	EepromMcuReadBuffer(addr, puReadBuff, size);
	
	int i;
	
	printf("DDDDD start verify addr %x, size %d \n", addr, size);
	for(i = 0; i < size; i ++){
		if(buffer[i] != puReadBuff[i]){
			printf("DDDDDDDDDD Verify faile at %d \n", i);
		}
	}
#endif

	return SUCCESS;
}


void EepromMcuSetDeviceAddr( uint8_t addr )
{
    assert_param( FAIL );
}

uint8_t EepromMcuGetDeviceAddr( void )
{
    assert_param( FAIL );
    return 0;
}


