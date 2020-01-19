/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Aug 2, 2019
 *      Author: SHARATH
 */

#include "stm32f446xx_gpio_driver.h"
#include <stdint.h>


/** @fn
  * @brief
  * @param
  *
  * @param
  *
  * @retval
  */

/*
 * local helper functions
 */

static void GPIO_PeriClockControl(GPIO_Typedef *pGPIOx, uint8_t EnoDi);



/*
 * GPIO Initialization
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//Peripheral clock enable

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);



	uint32_t temp=0;
	// 1.configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANLG)
	{
			temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear the required bits before setting
			pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else
	{
		//1.Configure Interrupt modes
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			 //1.configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); //enable FTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//disable RTSR

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//2.Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//3.Configure both FTFR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		//2.configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_port_code(pGPIOHandle->pGPIOx);
		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4 );

		//3. enable interrupt delivery from pheri to processor
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );


	}
	temp=0;
	// 2.configure the output type
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOutType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear the required bits before setting
	pGPIOHandle->pGPIOx->OTYPER|=temp;

		temp=0;
	// 3.GPIO port output speed register
	    temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear the required bits before setting
		pGPIOHandle->pGPIOx->OSPEEDR|=temp;

		temp=0;
	// 4. configure GPIO port pull-up/pull-down register
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear the required bits before setting
			pGPIOHandle->pGPIOx->PUPDR|=temp;

			temp=0;
	// 5. configure the alt function setting

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT)
		{
			uint32_t temp1,temp2=0;
			uint32_t temp3=0;
			//configure alternate function registers with the value mentioned in @GPIO_PinAltFunMode
			temp1=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8;
			temp2=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // clear the required bits before setting
			temp3 |=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
			pGPIOHandle->pGPIOx->AFR[0] |= temp3;
			temp3=0;
		}



}



void GPIO_DeInit(GPIO_Typedef *pGPIOx)
{
	     if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC)
					{
					GPIOC_REG_RESET();
					}
			else if(pGPIOx == GPIOD)
					{
					GPIOD_REG_RESET();
					}
			else if(pGPIOx == GPIOE)
					{
					GPIOE_REG_RESET();
					}
			else if(pGPIOx == GPIOF)
					{
					GPIOF_REG_RESET();
					}
			else if(pGPIOx == GPIOG)
					{
					GPIOG_REG_RESET();
					}
			else if(pGPIOx == GPIOH)
					{
					GPIOH_REG_RESET();
					}
			else if(pGPIOx == GPIOI)
					{
					GPIOI_REG_RESET();
					}


}
/*
 * GPIO Peripheral clock control
 */

static void GPIO_PeriClockControl(GPIO_Typedef *pGPIOx, uint8_t EnoDi)
{
	if(EnoDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		}
		else if(pGPIOx == GPIOC)
				{
					GPIOC_CLK_EN();
				}
		else if(pGPIOx == GPIOD)
				{
					GPIOD_CLK_EN();
				}
		else if(pGPIOx == GPIOE)
				{
					GPIOE_CLK_EN();
				}
		else if(pGPIOx == GPIOF)
				{
					GPIOF_CLK_EN();
				}
		else if(pGPIOx == GPIOG)
				{
					GPIOG_CLK_EN();
				}
		else if(pGPIOx == GPIOH)
				{
					GPIOH_CLK_EN();
				}
		else if(pGPIOx == GPIOI)
				{
					GPIOI_CLK_EN();
				}
	}
	else //if disable
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_CLK_DS();
				}
				else if(pGPIOx == GPIOB)
				{
					GPIOB_CLK_DS();
				}
				else if(pGPIOx == GPIOC)
						{
							GPIOC_CLK_DS();
						}
				else if(pGPIOx == GPIOD)
						{
							GPIOD_CLK_DS();
						}
				else if(pGPIOx == GPIOE)
						{
							GPIOE_CLK_DS();
						}
				else if(pGPIOx == GPIOF)
						{
							GPIOF_CLK_DS();
						}
				else if(pGPIOx == GPIOG)
						{
							GPIOG_CLK_DS();
						}
				else if(pGPIOx == GPIOH)
						{
							GPIOH_CLK_DS();
						}
				else if(pGPIOx == GPIOI)
						{
							GPIOI_CLK_DS();
						}
	}
}

/*
 * GPIO Read and Write
 */

uint8_t  GPIO_ReadFromInputPin(GPIO_Typedef *pGPIOx,uint8_t PinNumber)
{
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_Typedef *pGPIOx)
{
	uint16_t value;
	value =(uint16_t)(pGPIOx->IDR);
	return value;
}
void GPIO_WriteToOutputPin(GPIO_Typedef *pGPIOx,uint8_t PinNumber,uint8_t Value)
{

	if(Value== GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
void GPIO_WriteToOutputPort(GPIO_Typedef *pGPIOx,uint16_t Value)
{

 pGPIOx->ODR=Value;

}
void GPIO_ToggleOutputPin(GPIO_Typedef *pGPIOx,uint8_t PinNumber)
{

	pGPIOx->ODR ^= (1<<PinNumber); //toggle the bit feild
}
/*
 * GPIO IRQ configuration and handling for processor side
 */
void GPIO_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority )
{

	//find which IPR register, IPR0 ......IPR59.
	uint8_t ipr_n =IRQNumber/4;
	uint8_t ipr_section=IRQNumber%4;
	uint8_t total_shift = (ipr_section *8) +(8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_Intrpt_Prty_Reg + (ipr_n)) |= (IRQPriority << (total_shift));

}


void GPIO_IRQHandling(uint8_t PinNumber)
{

		if(EXTI->PR & (1 << PinNumber))
		{
			//CLEAR
			EXTI->PR |=(1 << PinNumber);
		}

}
