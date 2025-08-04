/*
 * stm32f446xx_I2C_driver.c
 *
 *  Created on: Apr 1, 2025
 *      Author: bright
 */
#include "stm32f446xx_I2C_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_PreScaler[8] = {2, 4, 8, 16};

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUF_EN);

	// Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVT_EN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUF_EN);

	// Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVT_EN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	// check for device mode
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		// device is in master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize == 1)
			{
				// first disable the ack
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

				// clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			// clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		// device is in slave mode
		// clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

uint32_t RCC_GetPLLOutputClock()
{
	// Not implemented
	return 0;
}
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t SystemClk;

	uint8_t ahbPreScaler, apb1PreScaler;
	uint8_t clksrc, ahbp, apb1;
	clksrc = ((RCC->CFGR >> 2) & 0x3);
	ahbPreScaler = (RCC->CFGR >> 4) & 0xF;
	apb1PreScaler = (RCC->CFGR >> 10) & 0x7;

	if (clksrc == 0)
	{
		SystemClk = 16000000;
	}
	else if (clksrc == 1)
	{
		SystemClk = 8000000;
	}
	else if (clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	(void)SystemClk;
	if (ahbPreScaler < 8)
	{
		ahbp = 1;
	}

	else
	{
		ahbp = AHB_PreScaler[ahbPreScaler - 8];
	}

	if (apb1PreScaler < 8)
	{
		apb1 = 1;
	}

	else
	{
		apb1 = APB1_PreScaler[apb1PreScaler - 8];
	}
	return SystemClk / (ahbp * apb1);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*API supported by this driver*/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		// mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_CYCLE_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE Configuration
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode

		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint32_t dummyRead;
	/*Generate Start condition set Start bit to 1*/
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	/*2.Confirm that the start generation is completed by checking the SB flag in the SR1*/
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG))
		;
	/*3.Send 7 bit of Slave address and 1 bit of  r/w bit - total 8*/
	SlaveAddr = SlaveAddr << 1; // 7 bit shift left by 1
	SlaveAddr |= 1;				// Clear 0. Bit for Read OP
	pI2CHandle->pI2Cx->DR = SlaveAddr;
	/**4.Confirm that the address is sent*/
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG))
		;

	if (len == 1)
	{
		// Disable ACK
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		// Clear ADDR
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
		// Wait to RXNE to be set
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG))
			;
		// Send Stop condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
		// Read data into the given pRxbuffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if (len > 1)
	{
		// Clear ADDR, ACK is 1 in this case
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
		for (uint32_t i = len; i > 0; i--)
		{
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG))
				;
			if (i == 2)
			{
				/* We have to send the stop when we read 1 byte before the last
				 *  So that there will be no extra byte coming from slave
				 *  NOTE: We read Read arried byte while we are receiving a new byte
				 */
				// About to terminate communication, ACK must be disable
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
				// Send Stop condition
				if (Sr == I2C_STOP_ENABLE)
				{
					pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
				}
			}

			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		// Enalbe ACK to normal state(HIGH)
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		// Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUF_EN);

		// Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVT_EN);

		// Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERR_EN);
	}

	return busystate;
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	/*1.Generate Start condition*/
	uint32_t dummyRead = 0;
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	/*2.Confirm that the start generation is completed by checking the SB flag in the SR1*/
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG))
		;
	/*3-Send 7 bit of Slave address and 1 bit of  r/w bit - total 8*/
	SlaveAddr = SlaveAddr << 1; // 7 bit shift left by 1
	SlaveAddr &= ~(1);			// Clear 0. Bit for write OP
	pI2CHandle->pI2Cx->DR = SlaveAddr;
	/**4.Confirm that the address is sent*/
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG))
		;
	/*5.Clear ADDR Flag by reading SR1 and SR2*/
	dummyRead = pI2CHandle->pI2Cx->SR1;
	dummyRead = pI2CHandle->pI2Cx->SR2;
	(void)dummyRead;
	/*6.Send Data till Len = 0*/
	while (len > 0)
	{
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG))
			; // Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}
	/*7.Wait for TXE=1 and BTF=1 before generating the STOP condition*/
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG))
		; // Wait till TXE is set
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG))
		; // Wait till BTF is set
	/*8.Send STOP Condition*/
	if (Sr == I2C_STOP_ENABLE)
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		// Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUF_EN);

		// Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVT_EN);

		// Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERR_EN);
	}

	return busystate;
}
/*Enable Clock of the given I2C*/
void I2C_PeripheralENABLE(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{

	if (ENorDI == ENABLE)
	{

		if (pI2Cx == pI2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == pI2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == pI2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{

		if (pI2Cx == pI2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == pI2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == pI2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		/*Interrupt Set-enable Register*/
		if (IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		/*Interrupt Clear-enable Register*/
		if (IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	/*Interrupt Priority Register*/
	*(NVIC_IPR_BASEADDR + (iprx * 4)) |= (IRQPriority << (8 * iprx_section + (8 - NO_PR_BITS_IMPLEMENTED)));
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
