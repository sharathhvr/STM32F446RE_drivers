/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Aug 11, 2019
 *      Author: SHARATH
 */
#include "stm32f446xx_spi_driver.h"
#include <stdint.h>

/*
 * local or private or helper functions
 */
static void	spi_ovr_error_interupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interupt_handle(SPI_Handle_t *pSPIHandle);
static void	spi_txe_interupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * SPI SOCE config
 */


void SPI_SSOEConfig(SPI_Typedef *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->SPI_CR2 |=(1 << SPI_CR2_SSOE);
		}
		if(EnorDi == DISABLE)
		{
			pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
		}
}


/*
 * SPI Peripheral Enable
 */


void SPI_ENABLE(SPI_Typedef *pSPIx, uint8_t EnoDi)
{

	if(EnoDi == ENABLE)
	{
		pSPIx->SPI_CR1 |=(1 << SPI_CR1_SPE);
	}
	if(EnoDi == DISABLE)
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}



/*
 * SPI Peripheral clock control
 */

void SPI_PeriClockControl(SPI_Typedef *pSPIx, uint8_t EnoDi)
{

	if(EnoDi == ENABLE)
		{
			if(pSPIx == SPI_1)
			{
				SPI1_CLK_EN();
			}
			else if(pSPIx == SPI_2)
			{
				SPI2_CLK_EN();
			}
			else if(pSPIx == SPI_3)
					{
				SPI3_CLK_EN();
					}
		}
		else //if disable
		{
			if(pSPIx == SPI_1)
			{
				SPI1_CLK_DS();
			}
			else if(pSPIx == SPI_2)
			{
				SPI2_CLK_DS();
			}
			else if(pSPIx == SPI_3)
			{
				SPI3_CLK_DS();
			}


		}
}

/*
 *  SPI Initialization and De-initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
 //Peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	//configure SPI_CR1 register
	uint32_t tempreg=0;

	//1.configure the SPI device mode
	tempreg |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	//2.configure the SPI Bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_MODE_FullDuplex)
	{
		//Bi-directional should be disabled
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_MODE_HalfDuplex)
	{
		//Bi-directional should be enabled
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_MODE_Simplex_RXonly)
	{
		//Bi-directional should be disabled and
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RX only should be set
		tempreg |= (1 << SPI_CR1_RXONLY);

	}

	//3.Configure SPI clock
		tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	//4.configure the DFF
		tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	//5.configure CPOL
		tempreg|=  (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	//6.configure CPHA
		tempreg|= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	//7.configure SSM
		tempreg|= (pSPIHandle->SPI_Config.SPI_SSM<< SPI_CR1_SSM);

	//7.configure SSI
			tempreg|= (pSPIHandle->SPI_Config.SPI_SSI<< SPI_CR1_SSI);

	pSPIHandle->pSPIx->SPI_CR1 =tempreg;
}



void SPI_DeInit(SPI_Typedef *pSPIx)
{

	if(pSPIx ==SPI_1)
	{

		RCC->APB2RSTR |= (1 << 12); //set Bit 12 of APB2RSTR
	}
	else if(pSPIx == SPI_2)
	{
		RCC->APB1RSTR |= (1 << 14); //set Bit 14 of APB1RSTR
	}
	else if(pSPIx==SPI_3)
	{
		RCC->APB1RSTR |= (1 << 15); //set Bit 15 of APB2RSTR
	}


}

/*
 * SPIO get flag status
 */

uint8_t SPI_getFLAG_status(SPI_Typedef *pSPIx,uint8_t Flag_name)
{

	return (pSPIx->SPI_SR & (1 << Flag_name));
}

/*
 * SPIO Data send and receive
 */

//blocking call (polling type)

void SPI_DataSEND(SPI_Typedef *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{

	while(Len != 0)
	{
		//1.wait if SPIx_SR TXE=0;
		while(SPI_getFLAG_status(pSPIx,SPI_TXE_FLAG) == DISABLE);

		//2. check if DFF=1 or 0
		if((pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)))
		{
			//Load 16-bit data onto data register
			pSPIx->SPI_DR= *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//Load 8-bit data onto data register
			pSPIx->SPI_DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}

	}

}
void SPI_DataRECEIVE(SPI_Typedef *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{

	while(Len != 0)
		{
			//1.wait if SPIx_SR RXE=0;
			//while((pSPIx->SPI_SR & (1 << 0)) ); or
			while(!(SPI_getFLAG_status(pSPIx,SPI_SR_RXNE)));
			//2. check if DFF=1 or 0
			if((pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)))
			{
				//Load 2 byte data into Rx buffer
				*((uint16_t*)pRxBuffer)=pSPIx->SPI_DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}
			else
			{
				//Load 1 byte data into Rx buffer
				*(pRxBuffer)=pSPIx->SPI_DR;
				Len--;
				pRxBuffer++;
			}

		}
}

/*
 * SPI IRQ configuration and handling
 */
void SPI_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				// set ISERO register
				*NVIC_ISER0_BASEADDR |= (1 << IRQNumber);

			}else if(IRQNumber > 31 && IRQNumber < 64)
			{
				// set ISER1 register
				*NVIC_ISER1_BASEADDR |= (1 << (IRQNumber % 32));

			}else if(IRQNumber >= 64  && IRQNumber < 96)
			{
				// set ISER2 register
				*NVIC_ISER2_BASEADDR |= (1 << (IRQNumber % 64));
			}
		}
		else  //DISABLE
		{
			if(IRQNumber <= 31)
					{
						// clear ICERO register
					*NVIC_ICER0_BASEADDR |= (1 << IRQNumber);

					}else if(IRQNumber > 31 && IRQNumber < 64)
					{
						// clear ICER1 register
					*NVIC_ICER1_BASEADDR |= (1 << (IRQNumber % 32));
					}else if(IRQNumber >= 64  && IRQNumber < 96)
					{
						// clear ICER2 register
					*NVIC_ICER2_BASEADDR |= (1 << (IRQNumber % 64));
					}
		}

}

/*
 * SPI IRQ priority configuration
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority )
{

	//find which IPR register, IPR0 ......IPR59.
		uint8_t ipr_n =IRQNumber/4;
		uint8_t ipr_section=IRQNumber%4;
		uint8_t total_shift = (ipr_section *8) +(8 - NO_PR_BITS_IMPLEMENTED);
		*(NVIC_Intrpt_Prty_Reg + (ipr_n)) |= (IRQPriority << (total_shift));


}

/*
 * SPI IRQ handling
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//lets check if the interrupt was triggered by TXE
	temp1= pHandle->pSPIx->SPI_SR & (1<< SPI_SR_TXE);
	temp2=pHandle->pSPIx->SPI_CR2 & (1<< SPI_CR2_TXEIE);
	if(temp1 && temp2)   // or if((temp1 == 1) && (temp2 == 1))
	{
		//Handle transmission
		spi_txe_interupt_handle(pHandle);
	}

	//lets check if the interrupt was triggered by RXNE
		temp1= pHandle->pSPIx->SPI_SR & (1<< SPI_SR_RXNE);
		temp2=pHandle->pSPIx->SPI_CR2 & (1<< SPI_CR2_RXNEIE);
		if(temp1 && temp2)   // or if((temp1 == 1) && (temp2 == 1))
		{
			//Handle Reception
		spi_rxne_interupt_handle(pHandle);

		}

	//lets check if the interrupt was triggered by OVR
		temp1= pHandle->pSPIx->SPI_SR & (1<< SPI_SR_OVR);
		temp2=pHandle->pSPIx->SPI_CR2 & (1<< SPI_CR2_ERRIE);
		if(temp1 && temp2)   // or if((temp1 == 1) && (temp2 == 1))
		{
			//Handle Reception
			spi_ovr_error_interupt_handle(pHandle);

		}

}

/*
 * SPI SPI_DataSEND with interupt
 */

uint8_t SPI_DataSEND_IT(SPI_Handle_t *pSPI_Handle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state=pSPI_Handle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1.Save the Tx buffer address and Len info in some global variable

		pSPI_Handle->pTxBuffer=pTxBuffer;
		pSPI_Handle->TxLen=Len;

		//2.Mark the SPI state as busy  in transmission so that no other code can take over same SPI peripheral until tx is over
		pSPI_Handle->TxState= SPI_BUSY_IN_TX;

		//3.Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPI_Handle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);

	}

return state;


}

/*
 * SPI SPI_DataRECEIVE with interupt
 */

uint8_t SPI_DataRECEIVE_IT(SPI_Handle_t *pSPI_Handle,uint8_t *pRxBuffer, uint32_t Len)
{

	uint8_t state=pSPI_Handle->RxState;

		if(state != SPI_BUSY_IN_RX)
		{
			//1.Save the Tx buffer address and Len info in some global variable

			pSPI_Handle->pRxBuffer=pRxBuffer;
			pSPI_Handle->RxLen=Len;

			//2.Mark the SPI state as busy  in transmission so that no other code can take over same SPI peripheral until tx is over
			pSPI_Handle->RxState= SPI_BUSY_IN_TX;

			//3.Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPI_Handle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);

		}

	return state;
}


/*
 * helper functions
 */

static void	spi_txe_interupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{

		//Load 16-bit data onto data register
		pSPIHandle->pSPIx->SPI_DR= *((uint16_t*)(pSPIHandle->pTxBuffer));
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)(pSPIHandle->pTxBuffer)++;
	}
	else
	{
		//Load 8-bit data onto data register
		pSPIHandle->pSPIx->SPI_DR= *((pSPIHandle->pTxBuffer));
		pSPIHandle->TxLen--;
		(pSPIHandle->pTxBuffer)++;
	}

	if(!(pSPIHandle->TxLen) )
	{
		//if Length is 0 stop transmission

		SPI_closeTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);

	}

}

static void spi_rxne_interupt_handle(SPI_Handle_t *pSPIHandle)
{

	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{

			//Load 16-bit data onto data register
			pSPIHandle->pSPIx->SPI_DR= *((uint16_t*)(pSPIHandle->pRxBuffer));
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;
			(uint16_t*)(pSPIHandle->pRxBuffer)++;
		}
		else
		{
			//Load 8-bit data onto data register
			pSPIHandle->pSPIx->SPI_DR= *((pSPIHandle->pRxBuffer));
			pSPIHandle->RxLen--;
			(pSPIHandle->pRxBuffer)++;
		}

	if(!(pSPIHandle->RxLen) )
		{
			//if Length is 0 stop transmission
			SPI_closeTransmission(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);

		}

}


static void	spi_ovr_error_interupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1.clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp=pSPIHandle->pSPIx->SPI_DR;
		temp=pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;
	//2.inform the application that OVR error has occured
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


void SPI_ClearOVRFlag(SPI_Typedef  *pSPIx)
{
	uint8_t temp;
		//1.clear the OVR flag
			temp=pSPIx->SPI_DR;
			temp=pSPIx->SPI_SR;

		(void)temp;
}


void SPI_closeTransmission(SPI_Handle_t *pSPIHandle)
{
	//clear TXIE in CR2
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->TxLen=0;
	pSPIHandle->TxState=SPI_READY;
	pSPIHandle->pTxBuffer=NULL;
}

void SPI_closeReception(SPI_Handle_t *pSPIHandle)
{

	//clear RXNEIE in CR2
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->RxLen=0;
	pSPIHandle->RxState=SPI_READY;
	pSPIHandle->pRxBuffer=NULL;
}


//to be implemented in application hence made weak function
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t SPI_EVENT)
{

	//weak implementation, to be done in Application
}
