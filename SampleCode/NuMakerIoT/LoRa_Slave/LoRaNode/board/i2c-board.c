/****************************************************************************
 * @file     i2c-board.c
 * @version  V1.10
 * @brief    Target board I2C driver implementation
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"
#include "utilities.h"
#include "board-config.h"
#include "gpio.h"
#include "i2c-board.h"

#define MAX_I2C_INST	2

struct nu_i2c_modinit_s {
    uint32_t clkidx;
    uint32_t clksrc;
    uint32_t clkdiv;
    uint32_t rsetidx;    
    IRQn_Type irq_n;
    void *var;
};

typedef struct{
	I2C_T *i2c_base;
	I2cAddrSize I2cDataAddrSize;
}S_I2C_HANDLE;

static const struct nu_i2c_modinit_s i2c_modinit_tab[] = {
	{I2C0_MODULE, 0, MODULE_NoMsk, I2C0_RST, I2C0_IRQn, NULL},
	{I2C1_MODULE, 0, MODULE_NoMsk, I2C1_RST, I2C1_IRQn, NULL},
	{0, 0, 0, 0, (IRQn_Type) 0, NULL}
};

static S_I2C_HANDLE s_asI2CHandle[MAX_I2C_INST];


void I2cMcuInit( I2c_t *obj, I2cId_t i2cId, PinNames scl, PinNames sda )
{
	obj->I2cId = 0;
	
	if(i2cId >= MAX_I2C_INST)
	{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}	

	if(i2cId == I2C_1)
	{
		//Using I2C0 instance for I2C_1 id
		s_asI2CHandle[I2C_1].i2c_base = I2C0;

		//Set I2C multple function pin
		GpioInit( &obj->Scl, scl, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, I2C0_MFP_SCL_VALUE );
		GpioInit( &obj->Sda, sda, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, I2C0_MFP_SDA_VALUE );

		
	}
	else if(i2cId == I2C_2)
	{
		//Using I2C1 instance for I2C_2 id
		s_asI2CHandle[I2C_2].i2c_base = I2C1;

		//Set I2C multple function pin
		GpioInit( &obj->Scl, scl, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, I2C1_MFP_SCL_VALUE );
		GpioInit( &obj->Sda, sda, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, I2C1_MFP_SDA_VALUE );
	}
	else
	{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}
	
    CRITICAL_SECTION_BEGIN( );
	const struct nu_i2c_modinit_s *modinit = &i2c_modinit_tab[i2cId];

   // Reset this module
    SYS_ResetModule(modinit->rsetidx);

    // Select IP clock source
    CLK_SetModuleClock(modinit->clkidx, modinit->clksrc, modinit->clkdiv);
    // Enable IP clock
    CLK_EnableModuleClock(modinit->clkidx);
	
	obj->I2cId = i2cId;	
	s_asI2CHandle[obj->I2cId].I2cDataAddrSize = I2C_ADDR_SIZE_8;
	
    CRITICAL_SECTION_END( );
}

void I2cMcuFormat( I2c_t *obj, I2cMode mode, I2cDutyCycle dutyCycle, bool I2cAckEnable, I2cAckAddrMode AckAddrMode, uint32_t I2cFrequency )
{
	if(mode != MODE_I2C){
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}

	if(I2cAckEnable != true){
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}

	if(AckAddrMode != I2C_ACK_ADD_7_BIT){
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}

	I2C_Open(s_asI2CHandle[obj->I2cId].i2c_base, I2cFrequency);
}

void I2cMcuResetBus( I2c_t *obj )
{
	I2cMcuInit(obj, obj->I2cId, obj->Scl.pin, obj->Sda.pin);
}

void I2cMcuDeInit( I2c_t *obj )
{
	I2C_Close(s_asI2CHandle[obj->I2cId].i2c_base);

	GpioInit( &obj->Scl, obj->Scl.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	GpioInit( &obj->Sda, obj->Sda.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

void I2cSetAddrSize( I2c_t *obj, I2cAddrSize addrSize )
{
	s_asI2CHandle[obj->I2cId].I2cDataAddrSize = addrSize;
}

uint8_t I2cMcuWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
	uint32_t u32WriteLen;
	S_I2C_HANDLE *psI2CHandler = &s_asI2CHandle[obj->I2cId]; 
	
	if(psI2CHandler->I2cDataAddrSize == I2C_ADDR_SIZE_8){
		u32WriteLen = I2C_WriteMultiBytesOneReg(psI2CHandler->i2c_base, deviceAddr, addr, buffer, size); 
	}
	else{
		u32WriteLen = I2C_WriteMultiBytesTwoRegs(psI2CHandler->i2c_base, deviceAddr, addr, buffer, size); 
	}
	
	if(u32WriteLen != size)
		return FAIL;
	return SUCCESS;
}

uint8_t I2cMcuReadBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
	uint32_t u32ReadLen;
	S_I2C_HANDLE *psI2CHandler = &s_asI2CHandle[obj->I2cId]; 

	if(psI2CHandler->I2cDataAddrSize == I2C_ADDR_SIZE_8){
		u32ReadLen = I2C_ReadMultiBytesOneReg(psI2CHandler->i2c_base, deviceAddr, addr, buffer, size); 
	}
	else{
		u32ReadLen = I2C_ReadMultiBytesTwoRegs(psI2CHandler->i2c_base, deviceAddr, addr, buffer, size); 
	}

	if(u32ReadLen != size)
		return FAIL;
	return SUCCESS;
}

