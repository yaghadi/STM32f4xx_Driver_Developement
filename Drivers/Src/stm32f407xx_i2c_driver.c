/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jun 30, 2024
 *      Author: PurplE
 */

#include "stm32f407xx_i2c_driver.h"



static uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint8_t flagName);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle );
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi){
			if(pI2Cx==I2C1){
						I2C1_PCLK_EN();
			}
			if(pI2Cx==I2C2){
						I2C2_PCLK_EN();
					}
			if(pI2Cx==I2C3){
						I2C3_PCLK_EN();
			}
		}else{
			if(pI2Cx==I2C1){
						I2C1_PCLK_DI();
					}
					if(pI2Cx==I2C2){
								I2C2_PCLK_DI();
							}
					if(pI2Cx==I2C3){
								I2C3_PCLK_DI();
							}
		}
}

/*
 * Init and De-init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg=0;

	//Enable clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
	//ack control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << 10);
	pI2CHandle->pI2Cx->CR1 = tempreg;
	//configure the FREQ field of CR2
	tempreg=0;
	tempreg |= (RCC_GetPCKL1Value()/1000000U);
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);
	//program the device own address
	tempreg=0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress <<1);
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 |= tempreg;
	//CCR Calculations
	uint16_t CCR_Value =0;
	tempreg=0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//standard Mode
		CCR_Value=(RCC_GetPCKL1Value()/(2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
		tempreg |= (CCR_Value & 0xFFF);
	}else{
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		//Fast Mode
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			CCR_Value=(RCC_GetPCKL1Value()/(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
		}else{
			CCR_Value=(RCC_GetPCKL1Value()/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
		}
		tempreg |= (CCR_Value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR |= tempreg;
	tempreg =0;
	//trise Confifuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//standard Mode
		tempreg = (RCC_GetPCKL1Value()/1000000U)+1;
	}else{
		//Fast Mode
		tempreg = ((RCC_GetPCKL1Value()*300)/1000000000U)+1;
	}
	pI2CHandle->pI2Cx->TRISE |= (tempreg & 0x3F);
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
		if(pI2Cx==I2C1){
			I2C1_REG_RESET();
		}else if(pI2Cx==I2C2){
			I2C2_REG_RESET();
		}else if(pI2Cx==I2C3){
			I2C3_REG_RESET();
		}

}


/*
 * Data Send and Receive
 */
static uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint8_t flagName){
	if(pI2Cx->SR1 & flagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ControlACKing(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}


}
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}
void I2C_ControlACKing(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi){
	if(EnOrDi){
		pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
	}else{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
	}
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){

	//Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Complete the start generation is completed by checking the SB flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));
	//Send the address of the slave with r/nw bit set to W(0) (total 8bit)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);
	//confirm the address phase is completed by checking the ADDR flag bit in the SR1 register
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//clear ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CHandle);

	//Send the data until the len become 0
	while(Len >0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));//wait until txe is set
		pI2CHandle->pI2Cx->DR =*pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//when len become 0 wait for TXE=1 and BTF=1 Before generating the stop condition
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//Generate Stop Condition
	if(Sr==I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


}
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr){


	//Generate Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


	//Complete the start generation is completed by checking the SB flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//Send the address of the slave with r/nw bit set to W(1) (total 8bit)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	//confirm the address phase is completed by checking the ADDR flag bit in the SR1 register
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave
	if(Len==1){
		//disable Acking
		I2C_ControlACKing(pI2CHandle->pI2Cx, DISABLE);
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		//wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));
		//generate STOP Condition
		if(Sr==I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		//read data in to buffer
		*pRxBuffer=pI2CHandle->pI2Cx->DR;
	}
	//procedure to read data from slave when len >1
	if(Len >1){
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		//read the data until Len becomes zero
		for(uint32_t i=Len ;i>0;i--){
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));
			if(i==2){//if last 2 bytes are remaining
				//clear the ack bit
				I2C_ControlACKing(pI2CHandle->pI2Cx, DISABLE);
				//generate STOP condition
				if(Sr==I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			//read the data from register in to buffer
			*pRxBuffer=pI2CHandle->pI2Cx->DR;
			//increment the buffer address
			pRxBuffer++;
		}
	}
	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl==I2C_ACK_ENABLE){
		I2C_ControlACKing(pI2CHandle->pI2Cx, ENABLE);
	}
}
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxbuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr){
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;
}
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}


	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ControlACKing(pI2CHandle->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ControlACKing(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
    return (uint8_t) pI2C->DR;
}


/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi){
			if(IRQNumber <31){
				NVIC_ISER0 |= (1 <<IRQNumber);
			}
			else if(IRQNumber >31 && IRQNumber <64){
				NVIC_ISER1 |= (1 <<(IRQNumber%32));

			}else if (IRQNumber >63 && IRQNumber <96){
				NVIC_ISER2 |= (1 <<(IRQNumber%64));
			}
		}else{
			if(IRQNumber <31){
				NVIC_ICER0 |= (1 <<IRQNumber);
			}
			else if(IRQNumber >31 && IRQNumber <64){
				NVIC_ICER1 |= (1 <<(IRQNumber%32));

			}else if (IRQNumber >63 && IRQNumber <96){
				NVIC_ICER2 |= (1 <<(IRQNumber%64));
			}
		}

}
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
		//1. first lets find out the ipr register
		uint8_t iprx=IRQNumber/4;
		uint8_t iprx_section=IRQNumber%8;
		uint16_t shiftAmount=(8*iprx_section)+(8-NO_BITS_NOT_IMPLEM);
		*(NVIC_PR_BASE_ADDR +(iprx)) |= (IRQPriority << shiftAmount);
}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){

	//Interrupt handling for both master and slave mode of a device
	uint8_t temp1=pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	uint8_t temp2=pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	uint8_t temp3=pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3){
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}
	temp3=pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3){
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3=pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3){
		//BTF flag is set
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
			if(pI2CHandle->TxLen==0){
				//genrate stop condition
				if(pI2CHandle->Sr == I2C_DISABLE_SR)
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				//Reset all member elements of the handle structure
				I2C_CloseSendData(pI2CHandle);

				//notify the application about transmission complete
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
			}
		}else if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
			//do nothing
		}
	}
	temp3=pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if(temp1 && temp3){
		//STOPF flag is set
		//clear STOPF (i.e 1) read SR1 then write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	temp3=pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3){
		//TXE flag is set
		//check for Master/slave mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL )){
			if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}else
			{
				//slave
				//make sure that the slave is really in transmitter mode
			    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
			    {
			    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
			    }
			}
		}
	}
	temp3=pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3){
		//RXNE flag is set
		//check device mode .
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
