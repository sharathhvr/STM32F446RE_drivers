/*
 * 005SPI_testing.c
 *
 *  Created on: Aug 14, 2019
 *      Author: SHARATH
 */
#include <stm32f446xx.h>
#include <string.h>
void GPIO_SPI2_init(void)
{
	GPIO_Handle_t GPIO_SPI2;
	GPIO_SPI2.pGPIOx=GPIOB;
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinAltFunMode= 5;
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALT;
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinOutType=GPIO_OP_PUSHPULL;
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPD_HGH;

	//SCLK
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN10;
	GPIO_Init(&GPIO_SPI2);


	//MOSI
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN15;
	GPIO_Init(&GPIO_SPI2);

	//MISO
 //GPIO_SPI2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN14;
	//GPIO_Init(&GPIO_SPI2);

	//NSS
	//GPIO_SPI2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN9;
	//GPIO_Init(&GPIO_SPI2);

}


void SPI2_Pheri_Init()
{

	SPI_Handle_t SPI2_Config;
	SPI2_Config.pSPIx=SPI_2;
	SPI2_Config.SPI_Config.SPI_BusConfig=SPI_BUS_MODE_FullDuplex;
	SPI2_Config.SPI_Config.SPI_CPHA=SPI_CPHA_LOW;
	SPI2_Config.SPI_Config.SPI_CPOL=SPI_CPOL_LOW;
	SPI2_Config.SPI_Config.SPI_DFF=SPI_DFF_8BIT;
	SPI2_Config.SPI_Config.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2_Config.SPI_Config.SPI_SSM=SPI_SSM_EN;
	SPI2_Config.SPI_Config.SPI_SclkSpeed=SPI_FPCLOCK_DIV8;
	SPI2_Config.SPI_Config.SPI_SSI=ENABLE;
	SPI_Init(&SPI2_Config);

}



int main(void)
{
	char user_data[] ="Hello World";

	GPIO_SPI2_init();  //GPIO initialization

	SPI2_Pheri_Init();  //SPI2 initialization

	//Enable SPI

	SPI_ENABLE(SPI_2,ENABLE);

	SPI_DataSEND(SPI_2,(uint8_t*)user_data,strlen(user_data));
	//lets confirm SPI is not busy
		while( SPI_getFLAG_status(SPI_2,SPI_BSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_ENABLE(SPI_2,DISABLE);
	while(1);
	return 0;
}
