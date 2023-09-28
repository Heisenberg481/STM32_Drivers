/*
 * stm32f0308_spi_driver.h
 *
 *  Created on: Sep 28, 2023
 *      Author: AndriyB
 */

#ifndef INC_STM32F0308_SPI_DRIVER_H_
#define INC_STM32F0308_SPI_DRIVER_H_

#include "STM32F0308.h"
//Configuration structure for SPIx peripheral
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

// Handle structure for SPIx peripheral
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;


// Spi repripharal clock
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
//Init and deInit SPI
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
//Data Send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
//IRQ Configuration and ISR handling
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfid (uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);











#endif /* INC_STM32F0308_SPI_DRIVER_H_ */
