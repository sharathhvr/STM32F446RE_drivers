/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Aug 2, 2019
 *      Author: SHARATH
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"
#include <stdint.h>

/*
 * This is a GPIO pin configuration structure
 */
typedef struct
{
	uint8_t GPIO_PinNumber;    // @Pin_Numbers_macros
	uint8_t GPIO_PinMode;    /*\< possible  values for pin_modes @GPIO_Pin_modes       >*/
	uint8_t GPIO_PinSpeed;    // @GPIO_speed_setting_macros
	uint8_t GPIO_PinPuPdControl; // @GPIO_port_pull-up/pull-down
	uint8_t GPIO_PinOutType;  //  @GPIO_output_type
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t ;


/*
 * This is a handle structure
 */

typedef struct
{
	GPIO_Typedef *pGPIOx; // This is to specify wich port is being configured
	GPIO_PinConfig_t GPIO_PinConfig; //pin configuration struct variable
}GPIO_Handle_t;


/* @GPIO_Pin_modes
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 	   0
#define GPIO_MODE_OUT	   1
#define GPIO_MODE_ALT      2
#define GPIO_MODE_ANLG     3
#define GPIO_MODE_IT_FT    4
#define GPIO_MODE_IT_RT	   5
#define GPIO_MODE_IT_RFT   6

/*
 * @GPIO_output_type
 */
#define GPIO_OP_PUSHPULL   0
#define GPIO_OP_OPENDRAIN  1

/*
 * @GPIO_speed_setting_macros
 */
#define GPIO_SPD_LOW		0
#define GPIO_SPD_MED		1
#define GPIO_SPD_HGH		2
#define GPIO_SPD_VHGH		3

/*
 * @GPIO_port_pull-up/pull-down
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2

/*
 * @Pin_Numbers_macros
 */
#define GPIO_PIN0		0
#define GPIO_PIN1		1
#define GPIO_PIN2		2
#define GPIO_PIN3		3
#define GPIO_PIN4		4
#define GPIO_PIN5		5
#define GPIO_PIN6		6
#define GPIO_PIN7		7
#define GPIO_PIN8		8
#define GPIO_PIN9		9
#define GPIO_PIN10		10
#define GPIO_PIN11		11
#define GPIO_PIN12		12
#define GPIO_PIN13		13
#define GPIO_PIN14		14
#define GPIO_PIN15		15

/*
 * @NVIC_IRQ_PRIORITY
 */
#define NVIC_PRIORITY_0		0
#define NVIC_PRIORITY_1		1
#define NVIC_PRIORITY_2		2
#define NVIC_PRIORITY_3		3
#define NVIC_PRIORITY_4		4
#define NVIC_PRIORITY_5		5
#define NVIC_PRIORITY_6		6
#define NVIC_PRIORITY_7		7
#define NVIC_PRIORITY_8		8
#define NVIC_PRIORITY_9		9
#define NVIC_PRIORITY_10	10
#define NVIC_PRIORITY_11	11
#define NVIC_PRIORITY_12	12
#define NVIC_PRIORITY_13	13
#define NVIC_PRIORITY_14	14
#define NVIC_PRIORITY_15	15



/***********************************************************************
 * 						API's supported by the driver
 *
 ***********************************************************************/

/*
 *  GPIO Initialization and De-initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Typedef *pGPIOx);

/*
 * GPIO Peripheral clock control
 */




/*
 * GPIO Read and Write
 */

uint8_t  GPIO_ReadFromInputPin(GPIO_Typedef *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Typedef *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Typedef *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_Typedef *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_Typedef *pGPIOx,uint8_t PinNumber);
/*
 * GPIO IRQ configuration and handling
 */
void GPIO_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority );
void GPIO_IRQHandling(uint8_t PinNumber);







#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
