/*
 * stm32f446xx_I2C_driver.h
 *
 *  Created on: Apr 1, 2025
 *      Author: bright
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_


#include "stddef.h"
#include "stdint.h"
#include "../../Driver/Inc/stm32f446xx.h"


typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_PinConfig_t;

typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_PinConfig_t I2C_Config;
	uint8_t read_complete;
	uint8_t target_reg;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t  DevAddr;
	uint32_t RxSize;
	uint8_t  Sr;
}I2C_Handle_t;

// I2C application states
#define I2C_READY 0
#define I2C_BUSY_IN_RX 1
#define I2C_BUSY_IN_TX 2
/*FLAG for I2C*/
#define I2C_SB_FLAG (1 << I2C_SR1_SB)
#define I2C_TXE_FLAG  (1 << I2C_SR1_TXE)
#define I2C_RXNE_FLAG (1 << I2C_SR1_RXNE)
#define I2C_SB_FLAG  (1 << I2C_SR1_SB)
#define I2C_BTF_FLAG  (1 << I2C_SR1_BTF)
#define I2C_ADDR_FLAG (1 << I2C_SR1_ADDR)

/*I2C_SCLSpeed*/
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_FM2K 200000

/*I2C_ACKControl*/
#define I2C_ACK_ENABLE 1
#define I2C_ACK_DISABLE 0

/*I2C_Fast Mode DC*/
#define I2C_FM_DUTY_CYCLE_2 0
#define I2C_FM_DUTY_CYCLE_16_9 1

/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

uint8_t  I2C_GetFlagStatus(I2C_RegDef_t *pSPIx, uint32_t FlagName);
/*API supported by this driver*/
void I2C_MPU_IRQHandling(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t *pRxBuffer);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
/*Enable Clock of the given I2C*/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);

/*IRQ Configuration and Handling*/
void I2C_IRQInterruptConfig(uint8_t IRQNumber , uint8_t ENorDI); /*Configure IRQ number of I2C*/
void I2C_IRQPriorityConfig(uint8_t IRQNumber , uint8_t IRQPriority); /*Configure IRQ number of I2C*/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_PeripheralENABLE(I2C_RegDef_t *pI2CX, uint8_t EnOrDi);/*Handle Interrupt*/
void I2C_ReceiveSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

// Application callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t Event);
#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
