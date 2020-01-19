/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Aug 26, 2019
 *      Author: SHARATH
 */

/*
 *    other I2C API's
 */

#include <stdint.h>
#include <stm32f446xx_i2c_driver.h>

uint16_t AHBPrescalerValues[9]={1,2,4,8,16,64,128,256,512};
uint8_t APBPrescalerValues[5]={1,2,4,8,16};


static void I2C_ExecuteAddressPhase(I2C_Regdef *pI2Cx, uint8_t SlaveAddr);
static void I2C_GenerateStartCondition(I2C_Regdef *pI2C);
static void I2C_GenerateStopCondition(I2C_Regdef *pI2C);
static void I2C_ClearAddrFlag(I2C_Regdef *pI2C);
uint32_t RCC_GetPclk1Value();


/*
 *   LOCAL FUNCTIONS OR HELPER FUNCTIONS
 */

// I2C generate stop condition

static void I2C_GenerateStopCondition(I2C_Regdef *pI2C)
{
	pI2C->I2C_CR1 |= (1 << I2C_CR1_STOP);

}

// I2C clear Addr flag

static void I2C_ClearAddrFlag(I2C_Regdef *pI2C)
{
	uint32_t dummyRead;
	dummyRead = pI2C->I2C_SR1;
	dummyRead = pI2C->I2C_SR1;
	(void)dummyRead;
}


// I2C execute address phase

static void I2C_ExecuteAddressPhase(I2C_Regdef *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1 << 0); //Slave Addr (7 bit) + Read bit
	pI2Cx->I2C_DR = SlaveAddr;

}


//I2C generate start condition helper function

static void I2C_GenerateStartCondition(I2C_Regdef *pI2C)
{
	pI2C->I2C_CR1 &= ~(1 << I2C_CR1_START);
	pI2C->I2C_CR1 |= (1 << I2C_CR1_START);

}


// get Peripheral clock

uint32_t RCC_GetPclk1Value()
{

	uint32_t sysclk,periclk;
	uint8_t temp1,temp2,temp3;
	uint16_t AHBPrescaler,APBPrescaler;

	temp1=(RCC->CFGR >> 0x2) & 0x3;
	if(temp1 == 0)
	{
		sysclk= 16000000;

	}
	else if(temp1 == 1)
	{
		sysclk= 8000000;
	}
	else if(temp1 == 2)
	{
		sysclk=0;
	}

	//find out AHB prescaler
	temp2=((RCC->CFGR >> 4) & 0xF);
	AHBPrescaler=AHBPrescalerValues[temp2-7];

	//find out APB1 prescaler
	temp3=((RCC->CFGR >> 10) & 0x7);
	APBPrescaler=APBPrescalerValues[temp3-3];

	//calculate APB1 clock value for I2C
	periclk=((sysclk/AHBPrescaler)/APBPrescaler);
	return periclk;
}




__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t I2C_EVENT)
{
	//implemented in application code



}



/*
 * I2C enable and disable
 */

void I2C_Enable_Disable(I2C_Regdef *pI2Cx, uint8_t EnoDi)
{
	if(EnoDi == ENABLE)
	{
			pI2Cx->I2C_CR1|= (1 << 0);
	}
	else if(EnoDi == DISABLE)
	{
		pI2Cx->I2C_CR1 &= ~(1 << 0);
	}
}


/*
 * I2C Peripheral clock control
 */

void I2C_PeriClockControl(I2C_Regdef *pI2Cx, uint8_t EnoDi)
{
	if(EnoDi == ENABLE)
			{
				if(pI2Cx == I2C_1)
				{
					I2C1_CLK_EN();
				}
				else if(pI2Cx == I2C_2)
				{
					I2C2_CLK_EN();
				}
				else if(pI2Cx == I2C_3)
						{
					I2C3_CLK_EN();
						}
			}
			else //if disable
			{
				if(pI2Cx == I2C_1)
				{
					I2C1_CLK_DS();
				}
				else if(pI2Cx == I2C_2)
				{
					I2C2_CLK_DS();
				}
				else if(pI2Cx == I2C_3)
				{
					I2C3_CLK_DS();
				}

		     }

}

/*
 *  I2C Initialization and De-initialization
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{

	uint32_t tempreg=0;
	//configure ACK control
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK );
	pI2CHandle->pI2C->I2C_CR1=tempreg;

	//configure FREQ in CR2
	tempreg=0;
	tempreg=RCC_GetPclk1Value()/1000000U;
	pI2CHandle->pI2C->I2C_CR2 =(tempreg & 0x3F);

	//configure the device own address
	tempreg=0;
	tempreg=(pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2C->I2C_OAR1=tempreg;

	/*
	 * Configuring CCR register
	 */

	uint16_t ccr_value=0;
	tempreg=0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//calculate ccr for standard mode
		ccr_value=(RCC_GetPclk1Value()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);

	}
	else
	{
		//first set fast mode

		tempreg |= (1 << 15);
		tempreg &= ~(1 << 14);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		//calculate ccr for fast mode
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value=(RCC_GetPclk1Value()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
			tempreg |= (ccr_value & 0xFFF);

		}
		else
		{
			ccr_value=(RCC_GetPclk1Value()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
			tempreg |= (ccr_value & 0xFFF);
		}
	}

	pI2CHandle->pI2C->I2C_CCR = tempreg;

	/*
	 * Configuring TRISE register
	 */

	tempreg=0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			tempreg= (RCC_GetPclk1Value()/1000000U)+1;
		}
		else
		{
			tempreg= (RCC_GetPclk1Value()*300/1000000000U)+1;
		}

	pI2CHandle->pI2C->I2C_TRISE= (tempreg & 0x3F) ;

}

void I2C_DeInit(I2C_Regdef *pI2Cx)
{

	if(pI2Cx ==I2C_1)
		{

			RCC->APB1RSTR |= (1 << 21); //set Bit 12 of APB2RSTR
		}
		else if(pI2Cx == I2C_2)
		{
			RCC->APB1RSTR |= (1 << 22); //set Bit 14 of APB1RSTR
		}
		else if(pI2Cx==I2C_3)
		{
			RCC->APB1RSTR |= (1 << 23); //set Bit 15 of APB2RSTR
		}


}



/*
 * I2C Data send
 */

void I2C_MasterSendData( I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr  )
{
	//1.Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2C);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	//  SB will be cleared once DR is written
	while(! I2C_getFLAG_status(pI2CHandle->pI2C,I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )

	I2C_ExecuteAddressPhase(pI2CHandle->pI2C, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1

	while(! I2C_getFLAG_status(pI2CHandle->pI2C,I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearAddrFlag(pI2CHandle->pI2C);

	//6. send the data until len becomes 0

	while( Len > 0)
	{
		while( ! I2C_getFLAG_status(pI2CHandle->pI2C,I2C_FLAG_TXE)); //wait till TXE is set
		pI2CHandle->pI2C->I2C_DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
		while( ! I2C_getFLAG_status(pI2CHandle->pI2C,I2C_FLAG_TXE));
		while( ! I2C_getFLAG_status(pI2CHandle->pI2C,I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
		I2C_GenerateStopCondition(pI2CHandle->pI2C);

}





/*
 * I2C Data receive
 */






/*
 * I2C Data send and receive (Interrupt)
 */





/*
 * I2C IRQ configuration and handling
 */





/*
 * I2C get flag status
 */

uint8_t I2C_getFLAG_status(I2C_Regdef *pI2Cx,uint8_t Flag_name)
{
	return (pI2Cx->I2C_CR1 & (1 << Flag_name));
}

