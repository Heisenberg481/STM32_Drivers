/*
 * STM32F0308.h
 *
 *  Created on: Jan 22, 2023
 *      Author: Nazar
 */

#ifndef INC_STM32F0308_H_
#define INC_STM32F0308_H_

#include <stdint.h>


// ARM Cortex M0 NVIC resgisters
#define NVIC_ISER0 			((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1 			((volatile uint32_t*)0xE000E104)

#define NVIC_ICER0 			((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1 			((volatile uint32_t*)0xE000E184)

#define NVIC_PR_BASE_ADDR  	((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED	4

#define FLASH_BASEADDR      0x08000000U
#define SRAM_BASEADDR       0x20000000U
#define ROM_BASEADDR        0x1FFFEC00U

#define APBPERIPH_BASE      0x40000000U
#define AHB1PERIPH_BASE     0x40020000U
#define AHB2PERIPH_BASE     0x48000000U

// AHB2 base addresses
#define GPIOA_BASEADDR      (AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR      (AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR      (AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR      (AHB2PERIPH_BASE + 0x0C00)
#define GPIOF_BASEADDR      (AHB2PERIPH_BASE + 0x1400)

// AHB1 base addresses
#define DMA_BASEADDR        (AHB1PERIPH_BASE + 0x0000)
#define RCC_BASEADDR        (AHB1PERIPH_BASE + 0x1000)
#define FLASHINT_BASEADDR   (AHB1PERIPH_BASE + 0x2000)
#define CRC_BASEADDR        (AHB1PERIPH_BASE + 0x3000)

// APB base addresses
#define TIM3_BASEADDR       (APBPERIPH_BASE + 0x0000)
#define TIM6_BASEADDR       (APBPERIPH_BASE + 0x1000)
#define TIM7_BASEADDR       (APBPERIPH_BASE + 0x1400)
#define TIM14_BASEADDR      (APBPERIPH_BASE + 0x2000)
#define RTC_BASEADDR        (APBPERIPH_BASE + 0x2800)
#define WWDG_BASEADDR       (APBPERIPH_BASE + 0x2C00)
#define IWGD_BASEADDR       (APBPERIPH_BASE + 0x3000)
#define SPI2_BASEADDR       (APBPERIPH_BASE + 0x3800)
#define USART2_BASEADDR     (APBPERIPH_BASE + 0x4400)
#define USART3_BASEADDR     (APBPERIPH_BASE + 0x4800)
#define USART4_BASEADDR     (APBPERIPH_BASE + 0x4C00)
#define USART5_BASEADDR     (APBPERIPH_BASE + 0x5000)
#define I2C1_BASEADDR       (APBPERIPH_BASE + 0x5400)
#define I2C2_BASEADDR       (APBPERIPH_BASE + 0x5800)
#define USB_BASEADDR        (APBPERIPH_BASE + 0x5C00)
#define USB_SRAM_BASEADDR   (APBPERIPH_BASE + 0x6000)
#define PWR_BASEADDR        (APBPERIPH_BASE + 0x7000)

#define SYSCFG_BASEADDR     (APBPERIPH_BASE + 0x10000)
#define EXTI_BASEADDR       (APBPERIPH_BASE + 0x10400)
#define USART6_BASEADDR     (APBPERIPH_BASE + 0x11400)
#define ADC_BASEADDR        (APBPERIPH_BASE + 0x12400)
#define TIM1_BASEADDR       (APBPERIPH_BASE + 0x12C00)
#define SPI1_BASEADDR       (APBPERIPH_BASE + 0x13000)
#define USART1_BASEADDR     (APBPERIPH_BASE + 0x13800)
#define TIM15_BASEADDR      (APBPERIPH_BASE + 0x14000)
#define TIM16_BASEADDR      (APBPERIPH_BASE + 0x14400)
#define TIM17_BASEADDR      (APBPERIPH_BASE + 0x14800)
#define DBGMCU_BASEADDR     (APBPERIPH_BASE + 0x15800)


// gpio structure
typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
	volatile uint32_t BRR;
}GPIO_RegDef_t;

//EXTI structure
typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

// RCC structure
typedef struct {
	volatile uint32_t RCC_CR;
	volatile uint32_t RCC_CFGR;
	volatile uint32_t RCC_CIR;
	volatile uint32_t RCC_APB2RSTR;
	volatile uint32_t RCC_APB1RSTR;
	volatile uint32_t RCC_AHBENR;
	volatile uint32_t RCC_APB2ENR;
	volatile uint32_t RCC_APB1ENR;
	volatile uint32_t RCC_BDCR;
	volatile uint32_t RCC_CSR;
	volatile uint32_t RCC_AHBRSTR;
	volatile uint32_t RCC_CFGR2;
	volatile uint32_t RCC_CFGR3;
	volatile uint32_t RCC_CR2;
}RCC_RegDef_t;

// SYSCFG structure
typedef struct {
	volatile uint32_t SYSCFG_CFGR1;
	volatile uint32_t SYSCFG_EXTICR[3];
	volatile uint32_t SYSCFG_CFGR2;
}SYSCFG_RegDef_t;


//SPI structure
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

//peripheral definitions
#define GPIOA				((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*) GPIOF_BASEADDR)


// rcc definition
#define RCC					((RCC_RegDef_t*) RCC_BASEADDR)
// EXTI
#define EXTI				((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*) SPI2_BASEADDR)

// Clock enable for gpio
#define GPIOA_PCLK_EN()		(RCC->RCC_AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()		(RCC->RCC_AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()		(RCC->RCC_AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()		(RCC->RCC_AHBENR |= (1 << 20))
#define GPIOF_PCLK_EN()		(RCC->RCC_AHBENR |= (1 << 22))

// Clock enable for i2c
#define I2C1_PLCK_EN() 		(RCC->RCC_APB1ENR |= (1 << 21))
//Clock enable for SPI
#define SPI1_PLCK_EN() 		(RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PLCK_EN() 		(RCC->RCC_APB1ENR |= (1 << 14))
// Clock enable for EXTI_SYSCFG
#define SYSCGF_EN() 		(RCC->RCC_APB2ENR |= (1 << 0))

// Clock disable for GPIO
#define GPIOA_PCLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 20))
#define GPIOF_PCLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 22))



//some macros
#define ENABLE 		1
#define DISABLE 	0
#define SET 		ENABLE
#define RESET 		DISABLE
#define GPIO_PIN_SET  SET
#define GPIO_PIN_RESET  RESET

#include "stm32f0308_gpio_driver.h"
#include "stm32f0308_spi_driver.h"

#endif /* INC_STM32F0308_H_ */
