/*
 * MPU_gy_521.h
 *
 *  Created on: Apr 10, 2025
 *      Author: bright
 */

#ifndef MPU_GY_521_H_
#define MPU_GY_521_H_


#include "../Driver/Inc/stm32f446xx.h"

#define MPU_ADDR0_LOW 0x68
#define MPU_ADDR0_HIGH 0x69


#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_INT_STATUS 0x3A
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C

#define MPU_REG_ACCEL_X_HIGH 0x3B
#define MPU_REG_ACCEL_X_LOW  0x3C
#define MPU_REG_ACCEL_Y_HIGH 0x3D
#define MPU_REG_ACCEL_Y_LOW  0x3E
#define MPU_REG_ACCEL_Z_HIGH 0x3F
#define MPU_REG_ACCEL_Z_LOW  0x40
#define MPU_REG_TEMP_HIGH   0x41
#define MPU_REG_TEMP_LOW    0x42
#define MPU_REG_GYRO_X_HIGH 0x43
#define MPU_REG_GYRO_X_LOW  0x44
#define MPU_REG_GYRO_Y_HIGH 0x45
#define MPU_REG_GYRO_Y_LOW  0x46
#define MPU_REG_GYRO_Z_HIGH 0x47
#define MPU_REG_GYRO_Z_LOW  0x48

#define MPU_GYRO_CONFIG_250 0
#define MPU_GYRO_CONFIG_500 1
#define MPU_GYRO_CONFIG_1000 2
#define MPU_GYRO_CONFIG_2000 3

#define MPU_ACCEL_CONFIG_2 0
#define MPU_ACCEL_CONFIG_4 1
#define MPU_ACCEL_CONFIG_8 2
#define MPU_ACCEL_CONFIG_16 3

#define MPU_READY 1
#define MPU_NOT_READY 0

typedef struct{
	GPIO_RegDef_t* sda_port_config;
	uint8_t sda_pin_config;
	GPIO_RegDef_t* scl_port_config;
	uint8_t scl_pin_config;
	uint8_t alt_mode;
	I2C_RegDef_t* pI2Cx;
	uint8_t MCU_addr_config;
	uint8_t gyro_config;
	uint8_t accel_config;

}MPU_Config_t;


typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    float temperature;     // in Celsius
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU_Data_t;


void MPU_i2c_inits(MPU_Config_t* MPU_Config);
MPU_Data_t MPU_Data_read();

#endif /* MPU_GY_521_H_ */
