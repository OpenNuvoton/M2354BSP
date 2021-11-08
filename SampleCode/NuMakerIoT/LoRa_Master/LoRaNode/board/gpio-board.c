/****************************************************************************
 * @file     gpio-board.c
 * @version  V1.10
 * @brief    Target board GPIO driver implementation
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"
#include "utilities.h"
#include "board-config.h"
#include "gpio.h"

#define EXTI_NUM_VECTORS	16

static Gpio_t *GpioIrq[EXTI_NUM_VECTORS];

void GpioMcuWrite(Gpio_t *obj, uint32_t value)
{
	int port_num = 0;

	if(obj == NULL){
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}

	if(obj->pin >= IOE_0){
		//TODO:
		return;
	}
	
	if(obj->pin == NC){
		return;
	}

	if(obj->port == PA)
		port_num = 0;
	else if(obj->port == PB)
		port_num = 1;
	else if(obj->port == PC)
		port_num = 2;
	else if(obj->port == PD)
		port_num = 3;
	else if(obj->port == PE)
		port_num = 4;
	else if(obj->port == PF)
		port_num = 5;
	else	
		return;
	
	GPIO_PIN_DATA_S(port_num, obj->pinIndex) = value;
}


uint32_t GpioMcuRead( Gpio_t *obj )
{
	int port_num = 0;

	if(obj == NULL){
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return 0;
	}

	if(obj->pin >= IOE_0){
		//TODO:
		return 0;
	}
	
	if(obj->pin == NC){
		return 0;
	}

	if(obj->port == PA)
		port_num = 0;
	else if(obj->port == PB)
		port_num = 1;
	else if(obj->port == PC)
		port_num = 2;
	else if(obj->port == PD)
		port_num = 3;
	else if(obj->port == PE)
		port_num = 4;
	else if(obj->port == PF)
		port_num = 5;
	else	
		return 0;
	
	return GPIO_PIN_DATA_S(port_num, obj->pinIndex);
}

void GpioMcuToggle(Gpio_t *obj)
{
	uint32_t pin_value;
	
	pin_value = GpioMcuRead(obj);

	if(pin_value){
		GpioMcuWrite(obj, 0);
	}
	else{
		GpioMcuWrite(obj, 1);
	}
}

void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
	int port_num = 0;
	volatile void *mfp_reg;
	volatile void *mfos_reg;
	
	if(pin >= IOE_0){
		//TODO:
		return;
	}
	
	obj->pin = pin;
	
	if(pin == NC)
		return;

	obj->pinIndex = (obj->pin & 0x0F);
	if((obj->pin & 0xF0) == 0x00){
		obj->port = PA;
		port_num = 0;
		mfos_reg = &SYS->GPA_MFOS;
		if(obj->pinIndex < 8)
			mfp_reg = &SYS->GPA_MFPL;
		else
			mfp_reg = &SYS->GPA_MFPH;
	}
	else if((obj->pin & 0xF0) == 0x10){
		obj->port = PB;
		port_num = 1;
		mfos_reg = &SYS->GPB_MFOS;
		if(obj->pinIndex < 8)
			mfp_reg = &SYS->GPB_MFPL;
		else
			mfp_reg = &SYS->GPB_MFPH;
	}
	else if((obj->pin & 0xF0) == 0x20){
		port_num = 2;
		obj->port = PC;
		mfos_reg = &SYS->GPC_MFOS;
		if(obj->pinIndex < 8)
			mfp_reg = &SYS->GPC_MFPL;
		else
			mfp_reg = &SYS->GPC_MFPH;
	}
	else if((obj->pin & 0xF0) == 0x30){
		port_num = 3;
		obj->port = PD;
		mfos_reg = &SYS->GPD_MFOS;
		if(obj->pinIndex < 8)
			mfp_reg = &SYS->GPD_MFPL;
		else
			mfp_reg = &SYS->GPD_MFPH;
	}
	else if((obj->pin & 0xF0) == 0x40){
		port_num = 4;
		obj->port = PE;
		mfos_reg = &SYS->GPE_MFOS;
		if(obj->pinIndex < 8)
			mfp_reg = &SYS->GPE_MFPL;
		else
			mfp_reg = &SYS->GPE_MFPH;
	}
	else if((obj->pin & 0xF0) == 0x50){
		port_num = 5;
		obj->port = PF;
		mfos_reg = &SYS->GPF_MFOS;
		if(obj->pinIndex < 8)
			mfp_reg = &SYS->GPF_MFPL;
		else
			mfp_reg = &SYS->GPF_MFPH;
	}
	else{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}

	uint32_t u32RegVal;
	uint32_t u32MFPPinOffset = obj->pinIndex % 8;
	
	u32RegVal = *((uint32_t *)mfp_reg);
	u32RegVal &= ~(0xF << (u32MFPPinOffset * 4));	//clear mfp, set GPIO mode
	
	if(mode == PIN_INPUT){
		//Set multiple function pin to GPIO
		*((uint32_t *)mfp_reg) = u32RegVal;

		GPIO_SetMode(obj->port, (1 << obj->pinIndex), GPIO_MODE_INPUT);

		if(type == PIN_NO_PULL)
			GPIO_SetPullCtl(obj->port, (1 << obj->pinIndex), GPIO_PUSEL_DISABLE);
		else if(type == PIN_PULL_UP)
			GPIO_SetPullCtl(obj->port, (1 << obj->pinIndex), GPIO_PUSEL_PULL_UP);
		else if(type == PIN_PULL_DOWN)
			GPIO_SetPullCtl(obj->port, (1 << obj->pinIndex), GPIO_PUSEL_PULL_DOWN);

	}
	else if(mode == PIN_OUTPUT){
		//Set multiple function pin to GPIO
		*((uint32_t *)mfp_reg) = u32RegVal;

		if(config == PIN_OPEN_DRAIN){
			GPIO_SetMode(obj->port, (1 << obj->pinIndex), GPIO_MODE_OPEN_DRAIN);

			if(type == PIN_PULL_UP)
				GPIO_SetPullCtl(obj->port, (1 << obj->pinIndex), GPIO_PUSEL_PULL_UP);
			else
				GPIO_SetPullCtl(obj->port, (1 << obj->pinIndex), GPIO_PUSEL_DISABLE);
		}
		else{
			GPIO_SetMode(obj->port, (1 << obj->pinIndex), GPIO_MODE_OUTPUT);
		}
	}
	else if(mode == PIN_ALTERNATE_FCT){
		//Set multiple function pin to alternate function
		u32RegVal |= value;
		*((uint32_t *)mfp_reg) = u32RegVal;
		
		u32RegVal = *((uint32_t *)mfos_reg);
		if(config == PIN_OPEN_DRAIN){
			u32RegVal |= (1 << obj->pinIndex);
		}
		else{
			u32RegVal &= ~(1 << obj->pinIndex);
		}

		*((uint32_t *)mfos_reg) = u32RegVal;
	}
	else{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}

	// Sets initial output value
	if(mode == PIN_OUTPUT){
		GpioMcuWrite(obj, value);
	}

}

void GpioMcuSetContext( Gpio_t *obj, void* context )
{
    obj->Context = context;
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
	uint32_t priority = 0;
	IRQn_Type IRQnb;
	uint32_t u32IntAttribs;
	
	if(obj->pin >= IOE_0){
		//TODO:
		return;
	}
	
	if(obj->pin == NC){
		return;
	}

	if(GpioIrq[obj->pinIndex] != NULL){
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;		
	}

	if(irqHandler == NULL)
	{
		return;
	}

	obj->IrqHandler = irqHandler;
	
	if(irqMode == IRQ_RISING_EDGE)
		u32IntAttribs = GPIO_INT_RISING;
	else if(irqMode == IRQ_FALLING_EDGE)
		u32IntAttribs = GPIO_INT_FALLING;
	else
		u32IntAttribs = GPIO_INT_BOTH_EDGE;

	switch( irqPriority )
	{
	case IRQ_VERY_LOW_PRIORITY:
	case IRQ_LOW_PRIORITY:
		priority = 3;
		break;
	case IRQ_MEDIUM_PRIORITY:
		priority = 2;
		break;
	case IRQ_HIGH_PRIORITY:
		priority = 1;
		break;
	case IRQ_VERY_HIGH_PRIORITY:
	default:
		priority = 0;
		break;
	}
	
	if(obj->port == PA){
		IRQnb = GPA_IRQn;
	}
	else if(obj->port == PB){
		IRQnb = GPB_IRQn;
	}
	else if(obj->port == PC){
		IRQnb = GPC_IRQn;
	}
	else if(obj->port == PD){
		IRQnb = GPD_IRQn;
	}
	else if(obj->port == PE){
		IRQnb = GPE_IRQn;
	}
	else if(obj->port == PF){
		IRQnb = GPF_IRQn;
	}
	else{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;		
	}

	GpioIrq[obj->pinIndex] = obj;
	GPIO_EnableInt(obj->port, (obj->pinIndex), u32IntAttribs);
	NVIC_SetPriority(IRQnb , priority);
	NVIC_EnableIRQ(IRQnb);
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
	IRQn_Type IRQnb;

	if(obj->pin >= IOE_0){
		//TODO:
		return;
	}

	if(obj->pin == NC){
		return;
	}

	if(obj->port == PA){
		IRQnb = GPA_IRQn;
	}
	else if(obj->port == PB){
		IRQnb = GPB_IRQn;
	}
	else if(obj->port == PC){
		IRQnb = GPC_IRQn;
	}
	else if(obj->port == PD){
		IRQnb = GPD_IRQn;
	}
	else if(obj->port == PE){
		IRQnb = GPE_IRQn;
	}
	else if(obj->port == PF){
		IRQnb = GPF_IRQn;
	}
	else{
		printf("ERROR! %s, %s, %d \n", __FILE__, __FUNCTION__, __LINE__);
		return;		
	}


	// Clear callback before changing pin mode
	GPIO_DisableInt(obj->port, (1 << obj->pinIndex));
	GpioIrq[obj->pinIndex] = NULL;
}

void Handle_GPIO_Irq(uint32_t line)
{
	if(line >= EXTI_NUM_VECTORS)
		return;
	
    if(( GpioIrq[line] != NULL ) && ( GpioIrq[line]->IrqHandler != NULL ))
    {
        GpioIrq[line]->IrqHandler( GpioIrq[line]->Context );
    }
}

void GPA_IRQHandler(void)
{
	int i;

	for(i = 0; i < 16; i ++){
		if(GPIO_GET_INT_FLAG(PA, 1 << i)){
			GPIO_CLR_INT_FLAG(PA, 1 << i);
			//Call gpio irq handler
			Handle_GPIO_Irq(i);
		}
	}	
}
void GPB_IRQHandler(void)
{
	int i;

	for(i = 0; i < 16; i ++){
		if(GPIO_GET_INT_FLAG(PB, 1 << i)){
			GPIO_CLR_INT_FLAG(PB, 1 << i);
			//Call gpio irq handler
			Handle_GPIO_Irq(i);
		}
	}	

}
void GPC_IRQHandler(void)
{
	int i;

	for(i = 0; i < 16; i ++){
		if(GPIO_GET_INT_FLAG(PC, 1 << i)){
			GPIO_CLR_INT_FLAG(PC, 1 << i);
			//Call gpio irq handler
			Handle_GPIO_Irq(i);
		}
	}	

}
void GPD_IRQHandler(void)
{
	int i;

	for(i = 0; i < 16; i ++){
		if(GPIO_GET_INT_FLAG(PD, 1 << i)){
			GPIO_CLR_INT_FLAG(PD, 1 << i);
			//Call gpio irq handler
			Handle_GPIO_Irq(i);
		}
	}	

}
void GPE_IRQHandler(void)
{
	int i;

	for(i = 0; i < 16; i ++){
		if(GPIO_GET_INT_FLAG(PE, 1 << i)){
			GPIO_CLR_INT_FLAG(PE, 1 << i);
			//Call gpio irq handler
			Handle_GPIO_Irq(i);
		}
	}	

}
void GPF_IRQHandler(void)
{
	int i;

	for(i = 0; i < 16; i ++){
		if(GPIO_GET_INT_FLAG(PF, 1 << i)){
			GPIO_CLR_INT_FLAG(PF, 1 << i);
			//Call gpio irq handler
			Handle_GPIO_Irq(i);
		}
	}
}
