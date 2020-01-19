/*
 * 003Extern_button.c
 *
 *  Created on: Aug 7, 2019
 *      Author: SHARATH
 */



#include "stm32f446xx.h"

void delay(void)
{
	for(int i= 0; i<500000/3; i++);
}


int main(void)
{

	GPIO_Handle_t led_toogle, GPIO_button;
	led_toogle.pGPIOx=GPIOA;
	led_toogle.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN8;
	led_toogle.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	led_toogle.GPIO_PinConfig.GPIO_PinOutType=GPIO_OP_PUSHPULL;
	//led_toogle.GPIO_PinConfig.GPIO_PinOutType=GPIO_OP_OPENDRAIN;
	//led_toogle.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	led_toogle.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	led_toogle.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPD_HGH;

	//before initializing GPIO enable clock
	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&led_toogle);

		GPIO_button.pGPIOx=GPIOB;
		GPIO_button.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN8;
		GPIO_button.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
		GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PD;
		GPIO_button.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPD_HGH;

		//before initializing GPIO enable clock
		GPIO_PeriClockControl(GPIOB,ENABLE);
		GPIO_Init(&GPIO_button);


	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB,GPIO_PIN8)== ENABLE )
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN8);

		}
		else{
		GPIO_WriteToOutputPin(GPIOA,GPIO_PIN8,DISABLE);
			}

	}
	return 0;
}


