/*
 * 006Arduino_SPI_TXonly.c
 *
 *  Created on: Aug 15, 2019
 *      Author: SHARATH
 */


/*
 * 005SPI_testing.c
 *
 *  Created on: Aug 14, 2019
 *      Author: SHARATH
 */
#include <stm32f446xx.h>
#include <string.h>

//command codes
#define COMMAND_LED_CTRL  		    0x50
#define COMMAND_SENSOR_READ  		0x51
#define COMMAND_LED_READ		    0x52
#define COMMAND_PRINT		        0x53
#define COMMAND_ID_READ		        0x54

#define LED_ON    1
#define LED_OFF   0

//arduino analog pins

#define ANALOG_PIN0			0
#define ANALOG_PIN1			1
#define ANALOG_PIN2			2
#define ANALOG_PIN3			3
#define ANALOG_PIN4			4

//LED PIN ARDUINO
#define LED_PIN        9
#define SENSOR_PIN

uint8_t SPI_slave_response_ACKorNACK(uint8_t *ACKbyte)
{
	if(*(ACKbyte) == 0xF5)
	{
		return 1;
	}
	else
		return 0;

}


void delay(void)
{
	for(int i= 0; i<500000/2; i++);
}

void GPIO_SPI2_init(void)
{
	GPIO_Handle_t GPIO_SPI2;
	GPIO_SPI2.pGPIOx=GPIOB;
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinAltFunMode= 5;
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALT;
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinOutType=GPIO_OP_PUSHPULL;
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU; //enable pull up when slave is connected
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPD_HGH;

	//SCLK
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN10;
	GPIO_Init(&GPIO_SPI2);


	//MOSI
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN15;
	GPIO_Init(&GPIO_SPI2);

	//MISO
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN14;
	GPIO_Init(&GPIO_SPI2);

	//NSS
	GPIO_SPI2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN9;
	GPIO_Init(&GPIO_SPI2);

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
	SPI2_Config.SPI_Config.SPI_SSM=SPI_SSM_DS;  //Hardware slave mngmt
	SPI2_Config.SPI_Config.SPI_SclkSpeed=SPI_FPCLOCK_DIV8;
	//SPI2_Config.SPI_Config.SPI_SSI=ENABLE; //used only when SSM =1 to avoid MODF
	SPI_Init(&SPI2_Config);

}

void GPIO__ButtonInit(void)
{
	GPIO_Handle_t GPIO_button;
	GPIO_button.pGPIOx=GPIOC;
	GPIO_button.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN13;
	GPIO_button.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_button.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPD_HGH;

	//before initializing GPIO enable clock
	GPIO_Init(&GPIO_button);
}

int main(void)
{

	uint8_t dummy_byte = 0xFF;
	uint8_t dummy_read;
	uint8_t ACKorNACKbyte;
	uint8_t argument[2];
	uint8_t ID_READ [11];

	GPIO__ButtonInit(); //initialize button

	GPIO_SPI2_init();  //GPIO initialization

	SPI2_Pheri_Init();  //SPI2 initialization

	//Enable SSOE

	SPI_SSOEConfig(SPI_2,ENABLE);

	while(1)
	{

			while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN13)); // check for button press
			delay();
			//Enable SPI, NSS goes low when SPI is enabled
			SPI_ENABLE(SPI_2,ENABLE);
		/*
		 *  LED control command
		 */
			//1. CMD_LED_CTRL <pin no>  <value(1)>
			uint8_t command_code = COMMAND_LED_CTRL;
			SPI_DataSEND(SPI_2,&command_code,1);

			//dummy read to clear RXNE
			SPI_DataRECEIVE(SPI_2,&dummy_read,1);

			//2.to receive ACK from the Arduino send a dummy byte
			SPI_DataSEND(SPI_2,&dummy_byte,1);

			//3.Receive SPI data on the master RX buffer
			SPI_DataRECEIVE(SPI_2,&ACKorNACKbyte,1);

			if(SPI_slave_response_ACKorNACK(&ACKorNACKbyte))
			{
				//send arguments
				argument[0]=LED_PIN;
				argument[1]=LED_ON;

				SPI_DataSEND(SPI_2,(uint8_t*)argument,2);
			}

			delay();
		/*
		 *  Sensor read command
         */
			//wait for button press

			while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN13)); // check for button press
			delay();
			//1.Send command Sensor read
			command_code= COMMAND_SENSOR_READ;
			SPI_DataSEND(SPI_2,&command_code,1);

			//2.Dummy read because the previous send resulted in an automatic reception from the slave
			SPI_DataRECEIVE(SPI_2,&dummy_read,1);

			//3.Dummy send to receive the result of command ie ACK or NACK
			SPI_DataSEND(SPI_2,&dummy_byte,1);

			//4.Receive SPI data on the master RX buffer
			SPI_DataRECEIVE(SPI_2,&ACKorNACKbyte,1);

			if(SPI_slave_response_ACKorNACK(&ACKorNACKbyte))
			{
				//send arguments
				argument[0]=ANALOG_PIN0;

				SPI_DataSEND(SPI_2,(uint8_t*)argument,1);

				//5.Dummy read to clear RXNE
				SPI_DataRECEIVE(SPI_2,&dummy_read,1);

				//the slave takes tike for ADC conversion hence dont Rx data immediately
				delay();
				//6.Dummy send to fetch sensor data from slave
				SPI_DataSEND(SPI_2,&dummy_byte,1);

				//7.Store thr RXed data on master to a variable
				uint8_t analog_read;
				SPI_DataRECEIVE(SPI_2,&analog_read,1);

			}

			delay();
			/*
			 *   read ID command
			*/
				//wait for button press

				while(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN13)); // check for button press
				delay();

				//1.Send command Sensor read
				command_code= COMMAND_ID_READ;
				SPI_DataSEND(SPI_2,&command_code,1);

				//2.Dummy read because the previous send resulted in an automatic reception from the slave
				SPI_DataRECEIVE(SPI_2,&dummy_read,1);

				//3.Dummy send to receive the result of command ie ACK or NACK
				SPI_DataSEND(SPI_2,&dummy_byte,1);

				//4.Receive SPI data on the master RX buffer
				SPI_DataRECEIVE(SPI_2,&ACKorNACKbyte,1);

				if(SPI_slave_response_ACKorNACK(&ACKorNACKbyte))
				{

					for(int i=0; i< 10; i++)
					{
						//send dummy data
						SPI_DataSEND(SPI_2,&dummy_byte,1);

						//Rx SPI data
						SPI_DataRECEIVE(SPI_2,&ID_READ[i],1);
					}

				}



			//lets confirm SPI is not busy
			while( SPI_getFLAG_status(SPI_2,SPI_BSY_FLAG));

			//Disable the SPI2 peripheral
			SPI_ENABLE(SPI_2,DISABLE);
	}

	return 0;
}







