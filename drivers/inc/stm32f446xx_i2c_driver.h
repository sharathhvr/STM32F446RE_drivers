/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: Aug 25, 2019
 *      Author: SHARATH
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include <stm32f446xx.h>
#include <stdint.h>



/*
 * I2C Config and handle
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint32_t I2C_DeviceAddress;
	uint32_t I2C_ACKControl;
	uint32_t I2C_FMDutyCycle;

}I2C_Config_t;


typedef struct
{
	I2C_Regdef *pI2C;
	I2C_Config_t I2C_Config;

}I2C_Handle_t;


/***********************************************************************
 * 						API's supported by the driver
 *
 ***********************************************************************/

/*
 *    other I2C API's
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t I2C_EVENT);


/*
 * I2C enable
 */

void I2C_Enable_Disable(I2C_Regdef *pI2Cx, uint8_t EnoDi);


/*
 * I2C Peripheral clock control
 */

void I2C_PeriClockControl(I2C_Regdef *pI2Cx, uint8_t EnoDi);

/*
 *  I2C Initialization and De-initialization
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Regdef *pI2Cx);



/*
 * I2C Data send
 */

void I2C_MasterSendData( I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr  );



/*
 * I2C Data receive
 */




/*
 * I2C IRQ configuration and handling
 */




/*
 * I2C get flag status
 */

uint8_t I2C_getFLAG_status(I2C_Regdef *pI2Cx,uint8_t Flag_name);





/*
 * @I2C_SCLSpeed options
 */

#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define I2C_SCL_SPEED_FM2K		200000

/*
 * @I2C_ACK options
 */

#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle options
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*
 * I2C Status register flags
 */

#define I2C_FLAG_SB				0
#define I2C_FLAG_ADDR			1
#define I2C_FLAG_BTF			2
#define I2C_FLAG_ADD10			3
#define I2C_FLAG_STOPF			4
#define I2C_FLAG_RXNE			6
#define I2C_FLAG_TXE			7
#define I2C_FLAG_BERR			8
#define I2C_FLAG_ARLO			9
#define I2C_FLAG_AF				10
#define I2C_FLAG_OVR			11
#define I2C_FLAG_PECERR			12
#define I2C_FLAG_TIMEOUT		14
#define I2C_FLAG_SMBALERT		15



#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
