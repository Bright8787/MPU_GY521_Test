/*
 * MPU_gy_521.c
 *
 *  Created on: Apr 10, 2025
 *      Author: bright
 */

#include "MPU_gy_521.h"
#include "stddef.h"
#include "string.h"

I2C_Handle_t MPU_Handle;

static void MPU_GPIO_config(MPU_Config_t *MPU_Config);
static void MPU_I2C_config(MPU_Config_t *MPU_Config);
static uint8_t MPU_Ready_status();
static void MPU_write_reg( uint8_t reg_addr, uint8_t MPU_addr, uint8_t value);
static void MPU_read_reg( uint8_t reg_addr, uint8_t MPU_addr, uint8_t* pRxBuffer);

void MPU_i2c_inits(MPU_Config_t* MPU_Config){

	MPU_GPIO_config(MPU_Config);
	MPU_I2C_config(MPU_Config);

	I2C_PeripheralENABLE(MPU_Handle.pI2Cx, ENABLE);
	MPU_Handle.pI2Cx->CR1 |= (1 << I2C_CR1_ACK);

	MPU_write_reg( MPU_REG_PWR_MGMT_1 , MPU_ADDR0_LOW , 0x00 );
	MPU_write_reg( MPU_REG_GYRO_CONFIG , MPU_ADDR0_LOW , MPU_Config->gyro_config );
	MPU_write_reg( MPU_REG_ACCEL_CONFIG, MPU_ADDR0_LOW , MPU_Config->accel_config);


}

static void MPU_ParseData(uint8_t *buffer, MPU_Data_t *data) {
    data->accel_x = (int16_t)(buffer[0] << 8 | buffer[1]);
    data->accel_y = (int16_t)(buffer[2] << 8 | buffer[3]);
    data->accel_z = (int16_t)(buffer[4] << 8 | buffer[5]);

    int16_t temp_raw = (int16_t)(buffer[6] << 8 | buffer[7]);
    data->temperature = (temp_raw / 340.0f) + 36.53f;  // Formula from datasheet

    data->gyro_x = (int16_t)(buffer[8]  << 8 | buffer[9]);
    data->gyro_y = (int16_t)(buffer[10] << 8 | buffer[11]);
    data->gyro_z = (int16_t)(buffer[12] << 8 | buffer[13]);
}

MPU_Data_t MPU_Data_read(){
	uint8_t tx[14];
	MPU_Data_t MPU_data;
	uint8_t reg_addr = MPU_REG_ACCEL_X_HIGH;
	while(!MPU_Ready_status());
	I2C_MasterSendData(&MPU_Handle, &reg_addr, 1 , MPU_ADDR0_LOW, I2C_STOP_DISABLE);
	I2C_MasterReceiveData(&MPU_Handle, tx , 14 , MPU_ADDR0_LOW, I2C_STOP_DISABLE);
	memset(&MPU_data,0,sizeof(MPU_data));
	MPU_ParseData(tx,&MPU_data);
	return MPU_data;
}


static void MPU_GPIO_config(MPU_Config_t *MPU_Config){

    GPIO_Handle_t SDA,SCL;

	memset(&SDA,0,sizeof(SDA));
	memset(&SCL,0,sizeof(SCL));
	/*Always initiate clock first before data line*/
	SCL.pGPIOx = MPU_Config->scl_port_config;
	SCL.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_ALTF;
	SCL.GPIO_PinConfig.GPIO_PinAltFunMode = MPU_Config->alt_mode;
	SCL.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;
	SCL.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	SCL.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SCL.GPIO_PinConfig.GPIO_PinNumber = MPU_Config->scl_pin_config;

	GPIO_Init(&SCL);
//	SDA.pGPIOx = MPU_Config->sda_port_config;
	SDA.pGPIOx = MPU_Config->sda_port_config;
	SDA.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_ALTF;
	SDA.GPIO_PinConfig.GPIO_PinAltFunMode = MPU_Config->alt_mode;
	SDA.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_OD;
	SDA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	SDA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SDA.GPIO_PinConfig.GPIO_PinNumber = MPU_Config->sda_pin_config;

	GPIO_Init(&SDA);


}

static void MPU_I2C_config(MPU_Config_t *MPU_Config){

	MPU_Handle.pI2Cx = MPU_Config->pI2Cx;
	MPU_Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	MPU_Handle.I2C_Config.I2C_DeviceAddress = MPU_Config->MCU_addr_config;
	MPU_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_CYCLE_2;
	MPU_Handle.I2C_Config.I2C_SCLSpeed =  I2C_SCL_SPEED_SM ;
	I2C_Init(&MPU_Handle);
}

static void MPU_write_reg( uint8_t reg_addr, uint8_t MPU_addr, uint8_t value){
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;
	I2C_MasterSendData(&MPU_Handle, tx , 2 , MPU_ADDR0_LOW, I2C_STOP_DISABLE);

}

static void MPU_read_reg( uint8_t reg_addr, uint8_t MPU_addr, uint8_t* pRxBuffer){
	I2C_MasterSendData(&MPU_Handle, &reg_addr , 1 , MPU_addr, I2C_STOP_DISABLE);
	I2C_MasterReceiveData(&MPU_Handle, pRxBuffer , 1 , MPU_addr, I2C_STOP_ENABLE);
}


static uint8_t MPU_Ready_status(){
	uint8_t ready = 0;

	MPU_read_reg(MPU_REG_INT_STATUS ,MPU_ADDR0_LOW, &ready);

	if((ready & 0x01)){
		return MPU_READY;
	}
	else{
		return MPU_NOT_READY;
	}

}



