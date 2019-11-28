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

TIM_HandleTypeDef tim_hal = {};
UART_HandleTypeDef huart = {};

int main(void)
{
	HAL_Init();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef gpio_hal = {};
	gpio_hal.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_hal.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOB, &gpio_hal);

	//TIM2
	__HAL_RCC_TIM2_CLK_ENABLE();
	tim_hal.Instance = TIM2;
	tim_hal.Init.Prescaler = SystemCoreClock/1000;
	tim_hal.Init.Period = 500;
	HAL_TIM_Base_Init(&tim_hal);

	HAL_TIM_Base_Start_IT(&tim_hal);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	// Konfiguracja PORTA pod UART
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef gpio_uart = {};
	gpio_uart.Mode = GPIO_AF3_USART2 | GPIO_AF7_USART2;
	gpio_uart.Pin = GPIO_PIN_15 | GPIO_PIN_2;
	HAL_GPIO_Init(GPIOA, &gpio_uart);

	__HAL_RCC_USART2_IS_CLK_ENABLED();
	huart.Instance = USART2;
	huart.Init.BaudRate = 115200;

	HAL_UART_MspInit(&huart);

	for(;;)
	{
		//HAL_UART_Transmit
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		HAL_Delay(300UL);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if(htim->Instance == TIM6)
	{
		HAL_IncTick();
	}
	else if(htim->Instance == TIM2)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	}
}



