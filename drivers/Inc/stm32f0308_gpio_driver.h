/*
 * stm32f0308_gpio_friver.h
 *
 *  Created on: Jan 22, 2023
 *      Author: Nazar
 */

#ifndef INC_STM32F0308_GPIO_DRIVER_H_
#define INC_STM32F0308_GPIO_DRIVER_H_

#include "STM32F0308.h"

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;
//pin numbers
#define GPIO_PIN_NO_0 		0
#define GPIO_PIN_NO_1 		1
#define GPIO_PIN_NO_2 		2
#define GPIO_PIN_NO_3 		3
#define GPIO_PIN_NO_4 		4
#define GPIO_PIN_NO_5 		5
#define GPIO_PIN_NO_6 		6
#define GPIO_PIN_NO_7 		7
#define GPIO_PIN_NO_8 		8
#define GPIO_PIN_NO_9 		9
#define GPIO_PIN_NO_10 		10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12 		12
#define GPIO_PIN_NO_13 		13
#define GPIO_PIN_NO_14 		14
#define GPIO_PIN_NO_15		15
// mode pin
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT 	4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6
// output mode
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1
// speed mode
#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM  	1
#define GPIO_SPEED_FAST  	2
#define GPIO_SPEED_HIGH 	3
// pull up/down resistors
#define GPIO_NO_PUPD 		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

//IRQ number
#define IRQ_NO_EXTI0_1		5
#define IRQ_NO_EXTI2_3		6
#define IRQ_NO_EXTI4_15		7

//GPIO reset macros
#define GPIOA_REG_RESET() 	do{(RCC->RCC_AHBRSTR |= (1 << 17)); (RCC->RCC_AHBRSTR &= ~(1 << 17));}while(0)
#define GPIOB_REG_RESET() 	do{(RCC->RCC_AHBRSTR |= (1 << 18)); (RCC->RCC_AHBRSTR &= ~(1 << 18));}while(0)
#define GPIOC_REG_RESET() 	do{(RCC->RCC_AHBRSTR |= (1 << 19)); (RCC->RCC_AHBRSTR &= ~(1 << 19));}while(0)
#define GPIOD_REG_RESET() 	do{(RCC->RCC_AHBRSTR |= (1 << 20)); (RCC->RCC_AHBRSTR &= ~(1 << 20));}while(0)
#define GPIOF_REG_RESET() 	do{(RCC->RCC_AHBRSTR |= (1 << 22)); (RCC->RCC_AHBRSTR &= ~(1 << 22));}while(0)

#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOF) ? 4 : 0)




void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *GPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutpuPort(GPIO_RegDef_t *GPIOx, uint16_t Value);

void GPIO_TogglePin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfid (uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
#endif /* INC_STM32F0308_GPIO_DRIVER_H_ */
