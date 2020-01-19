/*
 * 001Led_toggle.c
 *
 *  Created on: Aug 6, 2019
 *      Author: SHARATH
 */

#include "stm32f446xx.h"

void delay(void)
{
	for(int i= 0; i<500000; i++);
}


int main(void)
{

	GPIO_Handle_t led_toogle;
	led_toogle.pGPIOx=GPIOA;
	led_toogle.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN5;
	led_toogle.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	led_toogle.GPIO_PinConfig.GPIO_PinOutType=GPIO_OP_PUSHPULL;
	//led_toogle.GPIO_PinConfig.GPIO_PinOutType=GPIO_OP_OPENDRAIN;
	//led_toogle.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	led_toogle.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	led_toogle.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPD_HGH;

	//before initializing GPIO enable clock
	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&led_toogle);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN5);
		delay();
	}

	return 0;
}

