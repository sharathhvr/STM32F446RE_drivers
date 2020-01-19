/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Aug 11, 2019
 *      Author: SHARATH
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include <stm32f446xx.h>
#include <stdint.h>

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
	uint8_t SPI_SSI;

}SPI_Config_t;

typedef struct
{
	SPI_Typedef  *pSPIx;   //pointer to the particular SPI peripheral
	SPI_Config_t SPI_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t  TxState;
	uint8_t  RxState;

}SPI_Handle_t;



/*
 * @SPI_Device_Modes
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE       0

/*
 * @SPI_BUS_modes
 */
#define SPI_BUS_MODE_FullDuplex     	1
#define SPI_BUS_MODE_HalfDuplex			2
#define SPI_BUS_MODE_Simplex_RXonly     3

/*
 * @SPI clock configurations
 */
#define SPI_FPCLOCK_DIV2		0
#define	SPI_FPCLOCK_DIV4		1
#define SPI_FPCLOCK_DIV8		2
#define	SPI_FPCLOCK_DIV16		3
#define SPI_FPCLOCK_DIV32		4
#define	SPI_FPCLOCK_DIV64		5
#define SPI_FPCLOCK_DIV128		6
#define	SPI_FPCLOCK_DIV256		7

/*
 *  @SPI_DFF settings
 */

#define SPI_DFF_8BIT			0
#define SPI_DFF_16BIT			1

/*
 *  @SPI_CPOL
 */

#define SPI_CPOL_LOW			0
#define SPI_CPOL_HIGH			1

/*
 *  @SPI_CPHA
 */

#define SPI_CPHA_LOW			0
#define SPI_CPHA_HIGH			1

/*
 *  @SPI_SSM
 */

#define SPI_SSM_EN			1
#define SPI_SSM_DS			0

/*
 *  @SPI_SR status Flags bit position
 */
#define SPI_RXNE_FLAG			0
#define SPI_TXE_FLAG			1
#define SPI_CHSIDE_FLAG			2
#define SPI_UDR_FLAG			3
#define SPI_CRCERR_FLAG			4
#define SPI_MODF_FLAG			5
#define SPI_OVR_FLAG			6
#define SPI_BSY_FLAG			7
#define SPI_FRE_FLAG			8

/*
 *  possible SPI application states
 */

#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX 		2

/*
 *  possible SPI events
 */

#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4



/***********************************************************************
 * 						API's supported by the driver
 *
 ***********************************************************************/

/*
 *    other SPI API's
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t SPI_EVENT);
void SPI_ClearOVRFlag(SPI_Typedef  *pSPIx);
void SPI_closeTransmission(SPI_Handle_t *pSPIHandle);
void SPI_closeReception(SPI_Handle_t *pSPIHandle);

/*
 * SPI enable
 */

void SPI_ENABLE(SPI_Typedef *pSPIx, uint8_t EnoDi);


/*
 * SPI Peripheral clock control
 */

void SPI_PeriClockControl(SPI_Typedef *pSPIx, uint8_t EnoDi);

/*
 *  SPI Initialization and De-initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Typedef *pSPIx);



/*
 * SPIO Data send and receive
 */

void SPI_DataSEND(SPI_Typedef *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_DataRECEIVE(SPI_Typedef *pSPIx,uint8_t *pRxBuffer, uint32_t Len);

/*
 * SPIO Data send and receive
 */
uint8_t SPI_DataSEND_IT(SPI_Handle_t *pSPI_Handle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_DataRECEIVE_IT(SPI_Handle_t *pSPI_Handle,uint8_t *pRxBuffer, uint32_t Len);



/*
 * SPI IRQ configuration and handling
 */
void SPI_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority );
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * SPI get flag status
 */
uint8_t SPI_getFLAG_status(SPI_Typedef *pSPIx,uint8_t Flag_name);

/*
 * SPI SSOE config
 */

void SPI_SSOEConfig(SPI_Typedef *pSPIx, uint8_t EnorDi);



#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
