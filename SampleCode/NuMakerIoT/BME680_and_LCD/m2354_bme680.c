/**************************************************************************//**
 * @file     m2354_bme680.c
 * @version  V3.00
 * @brief    Initialize the BME680 device and related functions in M2354.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2019-2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "m2354_bme680.h"

#define BME680_I2C_PORT     I2C2
#define BME680_I2C_FREQ		100000


void m2354_delay_ms(uint32_t period)
{
    volatile uint32_t loop = period;
    
    while(loop--)
        CLK_SysTickDelay(1000); // 1 ms
}

static int8_t m2354_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	if(len > 1) 
    {
        I2C_ReadMultiBytesOneReg(BME680_I2C_PORT, dev_id, reg_addr, reg_data, len);
	}
    else 
    {
		*reg_data = I2C_ReadByteOneReg(BME680_I2C_PORT, dev_id, reg_addr);
	}

	return 0;
}

static int8_t m2354_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	if(len > 1) 
    {
        I2C_WriteMultiBytesOneReg(BME680_I2C_PORT, dev_id, reg_addr, reg_data, len);
	} 
    else 
    {
		I2C_WriteByteOneReg(BME680_I2C_PORT, dev_id, reg_addr, *reg_data);
	}
	
	return 0;
}

int8_t m2354_bme680_init(struct bme680_dev* dev)
{
	int8_t rslt;
	
	if (dev == NULL)
		return BME680_E_NULL_PTR;
	
    SYS_UnlockReg();
		
	/* Enable M2354 I2C module Clock */
    CLK_EnableModuleClock(I2C2_MODULE);
	
	/* Set PB.12 for I2C2_SDA and PB.13 for I2C2_SCL */
    SET_I2C2_SDA_PB12();
    SET_I2C2_SCL_PB13();
    	
    /* I2C1_SCL(PB.13) enable schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN13_Msk;

    /* Open I2C module and set bus clock */
    I2C_Open(BME680_I2C_PORT, BME680_I2C_FREQ);


	/* Assign BME680 I2C handler */
    dev->dev_id = BME680_I2C_ADDR_PRIMARY;
    dev->intf = BME680_I2C_INTF;
    dev->read = m2354_i2c_read;
    dev->write = m2354_i2c_write;
    dev->delay_ms = m2354_delay_ms;
    /* amb_temp can be set to 25 prior to configuring the gas sensor 
     * or by performing a few temperature readings without operating the gas sensor.
     */
    dev->amb_temp = 25;
	
	/* Initilize BME680 */
	rslt = bme680_init(dev);
	
	return rslt;
}

int8_t m2354_bme680_config(struct bme680_dev* dev)
{
    int8_t rslt;
    uint8_t set_required_settings;

    /* Set the temperature, pressure and humidity settings */
    dev->tph_sett.os_hum = BME680_OS_16X;
    dev->tph_sett.os_pres = BME680_OS_16X;
    dev->tph_sett.os_temp = BME680_OS_16X;
    dev->tph_sett.filter = BME680_FILTER_SIZE_7;

    /* Set the remaining gas sensor settings and link the heating profile */
    dev->gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    dev->gas_sett.heatr_temp = 320; /* degree Celsius */
    dev->gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    dev->power_mode = BME680_FORCED_MODE; 

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
        | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings, dev);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(dev);
	
	return rslt;
}
