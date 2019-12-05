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

#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_adc.h"

TIM_HandleTypeDef tim_hal = {};
UART_HandleTypeDef huart = {};
ADC_HandleTypeDef myAdc = {};

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
	tim_hal.Init.Period = 721;
	HAL_TIM_Base_Init(&tim_hal);

	HAL_TIM_Base_Start_IT(&tim_hal);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	// Konfiguracja PORTA pod UART
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef gpio_uart = {};
	gpio_uart.Mode = GPIO_MODE_AF_PP;
	gpio_uart.Alternate = GPIO_AF3_USART2;
	gpio_uart.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &gpio_uart);

	// Konfiguracja PORTA pod UART
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio_uart.Mode = GPIO_MODE_AF_PP;
	gpio_uart.Alternate = GPIO_AF7_USART2;
	gpio_uart.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOA, &gpio_uart);


	__HAL_RCC_USART2_CLK_ENABLE();
	huart.Instance = USART2;
	huart.Init.BaudRate = 115200;
	huart.Init.Mode = UART_MODE_TX;

	HAL_UART_Init(&huart);

	uint8_t pData[3];
	uint16_t dataSize = 1;
	uint32_t timeout = 200;


	// -------------------------------------------------------------
	//						ADC HAL
	// -------------------------------------------------------------

	//ADC_CHANNEL_TEMPSENSOR

	__HAL_RCC_ADC_CLK_ENABLE();
	myAdc.Instance = ADC1;
	myAdc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	myAdc.Init.Resolution = ADC_RESOLUTION_12B;
	HAL_ADCEx_Calibration_Start();




	for(;;)
	{
		//HAL_UART_Transmit
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		HAL_Delay(500UL);

		HAL_ADC_Start();
		HAL_ADC_PollForConversion();
		pData[0] = HAL_ADC_GetValue();
		HAL_UART_Transmit(&huart, pData, dataSize, timeout);
		HAL_ADC_Stop();
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



