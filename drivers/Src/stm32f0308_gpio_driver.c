/*
 * stm32f0308_gpio_driver.c
 *
 *  Created on: Jan 22, 2023
 *      Author: Nazar
 */
#include "stm32f0308_gpio_driver.h"
/****************************************************************
 * Documentation
 * fn name
 * what he does
 * description of parameters
 * what return
 * Note
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	 if ( EnorDi == ENABLE){
		 if (pGPIOx == GPIOA){
			 GPIOA_PCLK_EN();
		 }
		 else if (pGPIOx == GPIOB){
			 GPIOB_PCLK_EN();
		 }
		 else if (pGPIOx == GPIOC){
			 GPIOC_PCLK_EN();
		 }
		 else if (pGPIOx == GPIOD){
			 GPIOD_PCLK_EN();
		 }
		 else if (pGPIOx == GPIOF){
			 GPIOF_PCLK_EN();
		 }
	 }
	 else{
		 if (pGPIOx == GPIOA){
			 GPIOA_PCLK_DI();
		 }
		 else if (pGPIOx == GPIOB){
			 GPIOB_PCLK_DI();
		 }
		 else if (pGPIOx == GPIOC){
			 GPIOC_PCLK_DI();
		 }
		 else if (pGPIOx == GPIOD){
			 GPIOD_PCLK_DI();
		 }
		 else if (pGPIOx == GPIOF){
			 GPIOF_PCLK_DI();
		 }
	 }
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCGF_EN();
		SYSCFG->SYSCFG_EXTICR[temp1] = portcode << (temp2 * 4);
		// 3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint32_t pinOffset = 0;
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > 7){
			pinOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 7;
		}
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4 * pinOffset));
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
			pGPIOHandle->pGPIOx->AFRL |= temp;
		}
		else{
			pGPIOHandle->pGPIOx->AFRH |= temp;
		}
		pinOffset = 0;
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if (pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((GPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *GPIOx){
	uint16_t value;
	value = (uint16_t)(GPIOx->IDR);
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, uint8_t value){
	if (value == GPIO_PIN_SET){
		GPIOx->ODR |= (1 << PinNumber);
	}
	else{
		GPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutpuPort(GPIO_RegDef_t *GPIOx, uint16_t Value){
	GPIOx->ODR = Value;
}

void GPIO_TogglePin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber){
	GPIOx->ODR ^= (1 << PinNumber);
}


void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if (IRQNumber < 32){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber < 64){
			*NVIC_ISER1 |= (1<< (IRQNumber % 32));
		}
	}
	else{
		if (IRQNumber < 32){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber < 64){
			*NVIC_ICER1 |= (1<< (IRQNumber % 32));
		}
	}
}

void GPIO_IRQPriorityConfid (uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t iprx_sextion = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_sextion) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IRQNumber) |= IRQPriority << shift_amount;

}
void GPIO_IRQHandling(uint8_t PinNumber){
	 if(EXTI->PR & 1 << PinNumber){
		 EXTI->PR |= ( 1 << PinNumber);
	 }
}
