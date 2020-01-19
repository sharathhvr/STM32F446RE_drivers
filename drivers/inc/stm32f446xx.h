/*
 * stm32f446xx.h
 *
 *  Created on: Aug 1, 2019
 *      Author: SHARATH
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_


#define __weak __attribute__((weak))

#define __VO volatile
#define __IO volatile

#include <stddef.h>
#include <stdint.h>

/*
 * ARM Cortex M4 processor NVIC ISERx register addresses
 * These addresses are accessed directly hence convert them to pointers
 */
#define NVIC_ISER0_BASEADDR   ((__VO uint32_t*)0xE000E100)
#define NVIC_ISER1_BASEADDR   ((__VO uint32_t*)0xE000E104)
#define NVIC_ISER2_BASEADDR   ((__VO uint32_t*)0xE000E108)
#define NVIC_ISER3_BASEADDR   ((__VO uint32_t*)0xE000E10C)

/*
 * ARM Cortex M4 processor NVIC ICERx register addresses
 * These addresses are accessed directly hence convert them to pointers
 */
#define NVIC_ICER0_BASEADDR   ((__VO uint32_t*)0xE000E180)
#define NVIC_ICER1_BASEADDR   ((__VO uint32_t*)0xE000E184)
#define NVIC_ICER2_BASEADDR   ((__VO uint32_t*)0xE000E188)
#define NVIC_ICER3_BASEADDR   ((__VO uint32_t*)0xE000E18C)

/*
 * ARM Cortex M4 processor NVIC Priority register addresses
 * These addresses are accessed directly hence convert them to pointers
 */
#define NVIC_Intrpt_Prty_Reg ((__VO uint32_t*)0xE000E400)

/*
 * Number of bits implemented in NVIC's priority register section
 */
#define NO_PR_BITS_IMPLEMENTED  4


/*
 * base addresses of FLASH and SRAM
 */

#define FLASH_BASEADDR 		0x08000000U	/* Base Address of Flash memory  */
#define SRAM1_BASEADDR 		0x20000000U //112KB
#define SRAM2_BASEADDR 		0x20001C00U
#define ROM            		0x1FFF0000U
#define SRAM           		SRAM1_BASEADDR

/*
 * base addresses of Peripheral BUS. APBx ,AHBx
 */

#define PERIPH_BASE      	0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/*
 * base addresses of Peripherals attached to AHB1 bus
 */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + 0x2000)
#define RCC_BASEADDR        (AHB1PERIPH_BASE + 0x3800)

/*
 * base addresses of Peripherals attached to APB1 bus
 */

#define	I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define	I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define	I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00)
#define	SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define	SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)
#define	USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define	USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define	UART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00)
#define	UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)

/*
 * base addresses of Peripherals attached to APB2 bus
 */

#define	USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define	USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define	SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define	EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00)
#define	SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800)


/*************************************************************/
/*
 * Peripheral structure of Each peripheral
 */
/*************************************************************/

/*
 * Peripheral structure of I2Cx
 */
typedef struct
{
	__VO uint32_t I2C_CR1;
	__VO uint32_t I2C_CR2;
	__VO uint32_t I2C_OAR1;
	__VO uint32_t I2C_OAR2;
	__VO uint32_t I2C_DR;
	__VO uint32_t I2C_SR1;
	__VO uint32_t I2C_SR2;
	__VO uint32_t I2C_CCR;
	__VO uint32_t I2C_TRISE;
	__VO uint32_t I2C_FLTR;

}I2C_Regdef;



/*
 * Peripheral structure of SPIx
 */
typedef struct
{
	__VO uint32_t SPI_CR1;
	__VO uint32_t SPI_CR2;
	__VO uint32_t SPI_SR;
	__VO uint32_t SPI_DR;
	__VO uint32_t SPI_CRCPR;
	__VO uint32_t SPI_RXCRCR;
	__VO uint32_t SPI_TXCRCR;
	__VO uint32_t SPI_I2SCFGR;
	__VO uint32_t SPI_I2SPR;
}SPI_Typedef;

/*
 * Peripheral structure of GPIOx
 */

typedef struct
{
	__VO uint32_t MODER;
	__VO uint32_t OTYPER;
	__VO uint32_t OSPEEDR;
	__VO uint32_t PUPDR;
	__VO uint32_t IDR;
	__VO uint32_t ODR;
	__VO uint32_t BSRRL;
	__VO uint32_t BSRRH;
	__VO uint32_t LCKR;
	__VO uint32_t AFR[2];   //AFR[0] --->low register    AFR[1]----> High register
} GPIO_Typedef;



/*
 * Peripheral structure of RCC
 */

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  __IO uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
  __IO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
  __IO uint32_t CKGATENR;      /*!< RCC Clocks Gated ENable Register,                            Address offset: 0x90 */
  __IO uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */
} RCC_TypeDef;

/*
 * Peripheral structure of EXTI
 */

typedef struct
{
	__VO uint32_t IMR;
	__VO uint32_t EMR;
	__VO uint32_t RTSR;
	__VO uint32_t FTSR;
	__VO uint32_t SWIER;
	__VO uint32_t PR;

}EXTI_TypeDef;

/*
 * Peripherals structure for SYSCFG
 */

typedef struct
{
	__VO uint32_t MEMRMP;
	__VO uint32_t PMC;
	__VO uint32_t EXTICR[3];
	__VO uint32_t reserved1[2];
	__VO uint32_t CMPCR;
	//__VO uint32_t reserved2[2];
	//__VO uint32_t CFGR;
}SYSCFG_TypeDef;





/*
 * Peripherals type-casted to struct pointers
 */

#define I2C_1  ((I2C_Regdef*)I2C1_BASEADDR)
#define I2C_2  ((I2C_Regdef*)I2C2_BASEADDR)
#define I2C_3  ((I2C_Regdef*)I2C3_BASEADDR)

#define SPI_1  ((SPI_Typedef*)SPI1_BASEADDR )
#define SPI_2  ((SPI_Typedef*)SPI2_BASEADDR )
#define SPI_3  ((SPI_Typedef*)SPI3_BASEADDR )

#define SYSCFG ((SYSCFG_TypeDef*)SYSCFG_BASEADDR)

#define EXTI ((EXTI_TypeDef*)EXTI_BASEADDR)

#define RCC ((RCC_TypeDef*) RCC_BASEADDR)

#define GPIOA ((GPIO_Typedef*) GPIOA_BASEADDR)
#define GPIOB ((GPIO_Typedef*) GPIOB_BASEADDR)
#define GPIOC ((GPIO_Typedef*) GPIOC_BASEADDR)
#define GPIOD ((GPIO_Typedef*) GPIOD_BASEADDR)
#define GPIOE ((GPIO_Typedef*) GPIOE_BASEADDR)
#define GPIOF ((GPIO_Typedef*) GPIOF_BASEADDR)
#define GPIOG ((GPIO_Typedef*) GPIOG_BASEADDR)
#define GPIOH ((GPIO_Typedef*) GPIOH_BASEADDR)
#define GPIOI ((GPIO_Typedef*) GPIOI_BASEADDR)

/*
 * Clock enable macro for GPIOx peripheral
 */

#define GPIOA_CLK_EN()     (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()     (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()     (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()     (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()     (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()     (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()     (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()     (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN()     (RCC->AHB1ENR |= (1 << 8))

/*
 * Clock enable macro for I2Cx peripheral
 */

#define	I2C1_CLK_EN()     (RCC->APB1ENR |= (1 << 21))
#define	I2C2_CLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define	I2C3_CLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * Clock enable macro for SPIx peripheral
 */

#define	SPI2_CLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define	SPI3_CLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define	SPI1_CLK_EN()		(RCC->APB2ENR |= (1 << 12))

/*
 * Clock enable macro for USARTx peripheral
 */

#define	USART2_CLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define	USART3_CLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define	USART1_CLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define	USART6_CLK_EN()		(RCC->APB2ENR |= (1 << 5))

/*
 * Clock enable macro for SYSCFG peripheral
 */

#define	SYSCFG_CLK_EN()		(RCC->APB2ENR |= (1 << 14))

/*
 * Clock enable macro for UARTx peripheral
 */

#define	UART4_CLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define	UART5_CLK_EN()		(RCC->APB1ENR |= (1 << 20))

/*
 * Clock disable macro for GPIOx peripheral
 */

#define GPIOA_CLK_DS()     (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DS()     (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DS()     (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DS()     (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DS()     (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DS()     (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DS()     (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DS()     (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_CLK_DS()     (RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock disable macro for I2Cx peripheral
 */

#define	I2C1_CLK_DS()     (RCC->APB1ENR &= ~(1 << 21))
#define	I2C2_CLK_DS()		(RCC->APB1ENR &= ~(1 << 22))
#define	I2C3_CLK_DS()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock disable macro for SPIx peripheral
 */

#define	SPI2_CLK_DS()		(RCC->APB1ENR &= ~(1 << 14))
#define	SPI3_CLK_DS()		(RCC->APB1ENR &= ~(1 << 15))
#define	SPI1_CLK_DS()		(RCC->APB2ENR &= ~(1 << 12))

/*
 * Clock disable macro for USARTx peripheral
 */

#define	USART2_CLK_DS()		(RCC->APB1ENR &= ~(1 << 17))
#define	USART3_CLK_DS()		(RCC->APB1ENR &= ~(1 << 18))
#define	USART1_CLK_DS()		(RCC->APB2ENR &= ~(1 << 4))
#define	USART6_CLK_DS()		(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macro for SYSCFG peripheral
 */

#define	SYSCFG_CLK_DS()		(RCC->APB2ENR &= ~(1 << 14))

/*
 * Clock disable macro for UARTx peripheral
 */

#define	UART4_CLK_DS()		(RCC->APB1ENR &= ~(1 << 19))
#define	UART5_CLK_DS()		(RCC->APB1ENR &= ~(1 << 20))

/*
 * GPIOx port reset macros
 */
#define GPIOA_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<0));   (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<1));   (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<2));   (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<3));   (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<4));   (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<5));   (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<6));   (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<7));   (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<8));   (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/*
 * Some generic macros
 */

#define ENABLE		 	1
#define DISABLE 	 	0
#define SET          	ENABLE
#define RESET        	DISABLE
#define GPIO_PIN_SET  	ENABLE
#define GPIO_PIN_RESET 	DISABLE


/*
 *IRQ numbers
 */
#define EXTI0 			6
#define EXTI1 			7
#define EXTI2			8
#define EXTI3			9
#define EXTI4			10
#define EXTI9_5			23
#define EXTI15_10   	40
#define IRQno_SPI1		35
#define IRQno_SPI2		36
#define IRQno_SPI3		51


/*
 * port code geerator maco
 */

#define GPIO_port_code(x)      ((x == GPIOA) ? 0 :\
								(x == GPIOB) ? 1 :\
								(x == GPIOC) ? 2 :\
								(x == GPIOD) ? 3 :\
								(x == GPIOE) ? 4 :\
								(x == GPIOF) ? 5 :\
								(x == GPIOG) ? 6 :\
								(x == GPIOH) ? 7 :\
								(x == GPIOI) ? 8 :0)

/*
 * SPI bit definition macros
 */

//SPI_CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL        	1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR          	3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST    	7
#define SPI_CR1_SSI		    	8
#define SPI_CR1_SSM         	9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF         	11
#define SPI_CR1_CRCNXT			12
#define SPI_CR1_CRCEN        	13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE        15

//SPI_CR2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

//SPI_SR
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8


/*
 * I2C bit definition macros
 */

//I2C_CR1
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBYTE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

//I2C_CR2
#define I2C_CR2_FREQ0			0
#define I2C_CR2_FREQ1			1
#define I2C_CR2_FREQ2			2
#define I2C_CR2_FREQ3			3
#define I2C_CR2_FREQ4			4
#define I2C_CR2_FREQ5			5
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

//I2C_SR1
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

//I2C_SR2
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAUL		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC0			8
#define I2C_SR2_PEC1			9
#define I2C_SR2_PEC2			10
#define I2C_SR2_PEC3			11
#define I2C_SR2_PEC4			12
#define I2C_SR2_PEC5			13
#define I2C_SR2_PEC6			14
#define I2C_SR2_PEC7			15

//I2C_TRISE
#define I2C_TRISE_TRISE0		0
#define I2C_TRISE_TRISE1		1
#define I2C_TRISE_TRISE2		2
#define I2C_TRISE_TRISE3		3
#define I2C_TRISE_TRISE4		4
#define I2C_TRISE_TRISE5		5







#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

#endif /* INC_STM32F446XX_H_ */
