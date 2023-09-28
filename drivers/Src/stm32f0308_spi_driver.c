/*
 * stm32f0308_spi_driver.c
 *
 *  Created on: Sep 28, 2023
 *      Author: AndriyB
 */
#include "stm32f0308_spi_driver.h"

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	 if ( EnorDi == ENABLE){
			 if (pSPIx == SPI1){
				 SPI1_PLCK_EN();
			 }
			 else if (pSPIx == SPI2){
				 SPI2_PLCK_EN();
			 }
		 }
		 else{
			 if (pSPIx == SPI1){
				 SPI1_PLCK_EN();
			 }
			 else if (pSPIx == SPI2){
				 SPI2_PLCK_EN();
			 }
		 }
}
