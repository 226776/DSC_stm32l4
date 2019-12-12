#include "stm32l4xx.h"
			
#include "stm32l4xx_hal.h"

#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_usart.h"

ADC_HandleTypeDef myAdc = {};
TIM_HandleTypeDef tim_hal = {};

int main(void)
{
	HAL_Init();


	//TIM2
		__HAL_RCC_TIM2_CLK_ENABLE();
		tim_hal.Instance = TIM2;
		tim_hal.Init.Prescaler = SystemCoreClock/1000;
		tim_hal.Init.Period = 721;
		HAL_TIM_Base_Init(&tim_hal);

		HAL_TIM_Base_Start_IT(&tim_hal);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);


		//--------------------------------------------------------------------------------------------------
		//							CMSIS GPIO - DIODA LED_B3
		//--------------------------------------------------------------------------------------------------

		RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN_Msk;
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
		GPIOB->MODER &= ~GPIO_MODER_MODE3;
		GPIOB->MODER |= GPIO_MODER_MODE3_0;

		GPIOB->ODR = GPIO_ODR_OD3;


	//--------------------------------------------------------------------------------------------------
	//							UART (LL)
	//--------------------------------------------------------------------------------------------------
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

	LL_GPIO_InitTypeDef gpio_uart;
	LL_GPIO_StructInit(&gpio_uart);
	gpio_uart.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_uart.Alternate = LL_GPIO_AF_3;
	gpio_uart.Pin = LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOA, &gpio_uart);


	LL_GPIO_StructInit(&gpio_uart);
	gpio_uart.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_uart.Alternate = LL_GPIO_AF_7;
	gpio_uart.Pin = LL_GPIO_PIN_2;
	LL_GPIO_Init(GPIOA, &gpio_uart);


	LL_USART_InitTypeDef ll_uart;
	LL_USART_StructInit(&ll_uart);
	ll_uart.BaudRate = 115200;
	ll_uart.TransferDirection = LL_USART_DIRECTION_TX;

	LL_USART_Init(USART2, &ll_uart);
	LL_USART_Enable(USART2);



	// -------------------------------------------------------------
	//						ADC HAL
	// -------------------------------------------------------------

	//ADC_CHANNEL_TEMPSENSOR

	__HAL_RCC_ADC_CLK_ENABLE();
	myAdc.Instance = ADC1;
	myAdc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	myAdc.Init.Resolution = ADC_RESOLUTION_12B;
	//HAL_ADCEx_Calibration_Start(&myAdc);

	ADC_ChannelConfTypeDef hadc = {};
	hadc.Channel = ADC_CHANNEL_TEMPSENSOR;
	hadc.Rank = 1;
	hadc.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	HAL_ADC_ConfigChannel(&myAdc, &hadc);



	int32_t data;
	int32_t data2;
	uint8_t ud = 'T';

	for(;;)
	{
		//CMSIS toggle pin
		GPIOB->ODR ^=  GPIO_ODR_OD3;

		while(!LL_USART_IsActiveFlag_TXE(USART2));
		LL_USART_TransmitData8(USART2,ud);

		ud = 10;
		while(!LL_USART_IsActiveFlag_TXE(USART2));
		LL_USART_TransmitData8(USART2,ud);

		ud = 13;
		while(!LL_USART_IsActiveFlag_TXE(USART2));
		LL_USART_TransmitData8(USART2,ud);

		HAL_Delay(500UL);

		HAL_ADC_Start(&myAdc);
		if(HAL_ADC_PollForConversion(&myAdc, 100) == HAL_OK)
		{
			data = HAL_ADC_GetValue(&myAdc);
			HAL_ADC_Stop(&myAdc);
			data2 = (float)data * 3.3 / 4096;
			while(!LL_USART_IsActiveFlag_TXE(USART2));
			LL_USART_TransmitData8(USART2,(uint8_t)data2);
		}

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

	}
}



