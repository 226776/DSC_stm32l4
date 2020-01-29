/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

DAC_HandleTypeDef hdac = {};
TIM_HandleTypeDef tim_hal = {};

int main(void)
{
	HAL_Init();


	// ------------ diode init -------------
	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN_Msk;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER &= ~GPIO_MODER_MODE3;
	GPIOB->MODER |= GPIO_MODER_MODE3_0;

	GPIOB->ODR = GPIO_ODR_OD3;

	// ------------ DAC init -------------
	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN_Msk;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// PA4 analog mode DAC_OUT1
	GPIOA->MODER &= ~GPIO_MODER_MODE4;
	GPIOA->MODER |= GPIO_MODER_MODE4_0 | GPIO_MODER_MODE4_1;
	// PA5 analog mode DAC_OUT2
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= GPIO_MODER_MODE5_0 | GPIO_MODER_MODE5_1;


	__HAL_RCC_DAC1_CLK_ENABLE();

	//hdac.Instance = DAC1;






	while(1)
	{
		//HAL_Delay(500U);
		GPIOB->ODR ^= GPIO_ODR_OD3;
		for(int i = 0; i<100000; i++);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if(htim->Instance == TIM6)
	{
		HAL_IncTick();
	}
}
