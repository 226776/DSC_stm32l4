


#include "stm32l4xx.h"
			
#include "stm32l4xx_hal.h"

#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_usart.h"

ADC_HandleTypeDef myAdc = {};

int main(void)
{
	HAL_Init();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef gpio_hal = {};
	gpio_hal.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_hal.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOB, &gpio_hal);


	//--------------------------------------------------------------------------------------------------
	//							UART (LL)
	//--------------------------------------------------------------------------------------------------


	// Konfiguracja PORTA pod UART
	__HAL_RCC_GPIOA_CLK_ENABLE();		// Do przerobienia na LL_ ??

	LL_GPIO_InitTypeDef gpio_uart;
	LL_GPIO_StructInit(&gpio_uart);
	gpio_uart.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_uart.Alternate = LL_GPIO_AF_3;
	gpio_uart.Pin = LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOA, &gpio_uart);


	// Konfiguracja PORTA pod UART
	__HAL_RCC_GPIOA_CLK_ENABLE();		// Do przerobienia na LL_ ??

	LL_GPIO_StructInit(&gpio_uart);
	gpio_uart.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_uart.Alternate = LL_GPIO_AF_7;
	gpio_uart.Pin = LL_GPIO_PIN_2;
	LL_GPIO_Init(GPIOA, &gpio_uart);


	__HAL_RCC_USART2_CLK_ENABLE();		// Do przerobienia na LL_ ??

	LL_USART_InitTypeDef ll_uart;
	LL_USART_StructInit(&ll_uart);
	ll_uart.Init.BaudRate = 115200;
	ll_uart.Init.Mode = LL_USART_DIRECTION_TX;

	LL_USART_Init(USART2, &ll_uart);



	// -------------------------------------------------------------
	//						ADC HAL
	// -------------------------------------------------------------

	//ADC_CHANNEL_TEMPSENSOR

	__HAL_RCC_ADC_CLK_ENABLE();
	myAdc.Instance = ADC1;
	myAdc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	myAdc.Init.Resolution = ADC_RESOLUTION_12B;
	HAL_ADCEx_Calibration_Start();


	int32_t data;

	for(;;)
	{
		//HAL_UART_Transmit
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

		HAL_Delay(500UL);

		HAL_ADC_Start(&myAdc);
		if(HAL_ADC_PollForConversion(&myAdc, 100) == HAL_OK)
		{
			data = HAL_ADC_GetValue(&myAdc);
			HAL_ADC_Stop(&myAdc);
			data2 = sensorValue * ADC_REFERENCE_VOLTAGE_MV / ADC_MAX_OUTPUT_VALUE;

		}

	}
}



