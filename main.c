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
#include <math.h>

// Definitions
#define PI 3.1415

float angle = 0;
float scal = PI/180;
float sinVal = 0;
uint32_t DAC_data = 0;

// Structures declaration
DAC_HandleTypeDef hdac = {};
TIM_HandleTypeDef tim_hal = {};
ADC_HandleTypeDef hadc = {};

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
	GPIOA->MODER |= GPIO_MODER_MODE4_0;
	//GPIOA->MODER |= GPIO_MODER_MODE4_0 | GPIO_MODER_MODE4_1;
	// PA5 analog mode DAC_OUT2
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= GPIO_MODER_MODE5_0 | GPIO_MODER_MODE5_1;


	__HAL_RCC_DAC1_CLK_ENABLE();
	hdac.Instance = DAC1;
	HAL_DAC_Init(&hdac);

	DAC_ChannelConfTypeDef dac_conf = {};
	dac_conf.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	dac_conf.DAC_Trigger = DAC_TRIGGER_NONE;
	dac_conf.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	dac_conf.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	dac_conf.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	HAL_DAC_ConfigChannel(&hdac, &dac_conf, DAC_CHANNEL_2);

	// DAC Start
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);



	while(1)
	{

		sinVal = sin(angle*scal);
		sinVal *= 1000;
		sinVal += 2000;
		DAC_data = (uint32_t) sinVal;


		GPIOB->ODR ^= GPIO_ODR_OD3;
		GPIOA->ODR ^= GPIO_ODR_OD4;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1000);

		angle += 20;
		if(angle>360)
		{
			angle = 0;
		}

		HAL_Delay(10U);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if(htim->Instance == TIM6)
	{
		HAL_IncTick();
	}
}

