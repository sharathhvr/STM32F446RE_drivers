/*
 * 004Button_Interupt.c
 *
 *  Created on: Aug 10, 2019
 *      Author: SHARATH
 */

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
/*
 * LED STUFF
 */
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
/*
 * Button STUFF
 */
		GPIO_button.pGPIOx=GPIOB;
		GPIO_button.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN8;
		GPIO_button.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
		GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PD;
		GPIO_button.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPD_HGH;

		//before initializing GPIO enable clock
		GPIO_PeriClockControl(GPIOB,ENABLE);
		GPIO_Init(&GPIO_button);


		//Enable IRQ for GPIOB 8

		GPIO_IRQInteruptConfig(EXTI9_5,ENABLE);
		GPIO_IRQPriorityConfig(EXTI9_5,NVIC_PRIORITY_15);



	return 0;
}

void EXTI9_5_IRQHandler (void)
{
	GPIO_IRQHandling(GPIO_PIN8); //clearing pending event in the PR register
	GPIO_ToggleOutputPin(GPIOA,GPIO_PIN5);
}
