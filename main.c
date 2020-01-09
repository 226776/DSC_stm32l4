#include "stm32l4xx.h"
			
#include "stm32l4xx_hal.h"

#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_usart.h"
#include <stdio.h>

void sendData(uint8_t *data, uint8_t len);

ADC_HandleTypeDef myAdc = {};
TIM_HandleTypeDef tim_hal = {};
ADC_HandleTypeDef hadc_temp;
LL_GPIO_InitTypeDef gpio_uart;


int main(void)
{
	HAL_Init();



	__HAL_RCC_TIM2_CLK_ENABLE();
	tim_hal.Instance = TIM2;
	tim_hal.Init.Prescaler = SystemCoreClock/1000;
	tim_hal.Init.Period = 2000;
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


	hadc_temp.Instance = ADC1;
	hadc_temp.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc_temp.Init.Resolution = ADC_RESOLUTION_12B;
	hadc_temp.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc_temp.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc_temp.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc_temp.Init.LowPowerAutoWait = DISABLE;
	hadc_temp.Init.ContinuousConvMode = DISABLE;
	hadc_temp.Init.NbrOfConversion = 1;
	hadc_temp.Init.DiscontinuousConvMode = DISABLE;
	hadc_temp.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc_temp.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc_temp.Init.DMAContinuousRequests = DISABLE;
	hadc_temp.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc_temp.Init.OversamplingMode = DISABLE;
	HAL_ADC_Init(&hadc_temp);

	ADC_ChannelConfTypeDef ch_conf = {};

	ch_conf.Channel = ADC_CHANNEL_TEMPSENSOR;
	ch_conf.Rank = ADC_REGULAR_RANK_1;
	ch_conf.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	ch_conf.SingleDiff = ADC_SINGLE_ENDED;
	ch_conf.OffsetNumber = ADC_OFFSET_NONE;
	ch_conf.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc_temp, &ch_conf);

	HAL_ADCEx_Calibration_Start(&hadc_temp, 1);

	HAL_ADC_Start(&hadc_temp);

	uint8_t text[] = "Temperatura: ";
	uint32_t data;
	uint32_t Temperature;
	uint8_t temp[3];

	//HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	//HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
	//HAL_NVIC_EnableIRQ(ADC1_IRQn);

	//HAL_ADC_Start_IT(&hadc_temp);s



	for(;;)
	{
		HAL_Delay(500U);

		sendData(text, 13);


		if(HAL_ADC_PollForConversion(&hadc_temp, 100000) == HAL_OK)
		{

				data = HAL_ADC_GetValue(&hadc_temp);
				Temperature = __HAL_ADC_CALC_TEMPERATURE(3300, data, ADC_RESOLUTION_12B);
		}

		sprintf(temp,"%d",Temperature);
		sendData(temp, 2);
		sendData("\n",1);
		HAL_ADC_Start(&hadc_temp);


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
		GPIOB->ODR ^=  GPIO_ODR_OD3;
	}
}

void sendString(uint8_t *data, uint8_t len)
{
	uint8_t i;
	for (i = 0; i < len; i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART2));
		LL_USART_TransmitData8(USART2,data[i]);
	}
}

void sendData(uint8_t *data, uint8_t len)
{
	for(int i = 0; i<len; i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART2));
		LL_USART_TransmitData8(USART2,data[i]);
	}
}



