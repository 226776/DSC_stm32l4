
#include "stm32l4xx.h"

int main(void)
{

	//--------------------------------------------------------
	//	CMSIS GPIO - DIODA LED_B3
	//--------------------------------------------------------
	RCC -> AHB2ENR 	|= RCC_AHB2ENR_GPIOBEN;
	GPIOB -> MODER 	 = 0xFFFFFEBF;
	GPIOB -> MODER 	|= GPIO_MODER_MODE3_0;
	GPIOB -> MODER 	&= ~(GPIO_MODER_MODE3_1);
	GPIOB -> ODR 	|= GPIO_ODR_OD3;


	//--------------------------------------------------------
	//	MCO - RCC_CFGR
	//--------------------------------------------------------

	//RCC -> CFGR |= RCC_CFGR_MCOPRE_DIV1;
	//RCC -> CFGR |= RCC_CFGR_MCOPRE_DIV2;
	//RCC -> CFGR |= RCC_CFGR_MCOPRE_DIV4;
	//RCC -> CFGR |= RCC_CFGR_MCOPRE_DIV8;
	RCC -> CFGR |= RCC_CFGR_MCOPRE_DIV16;

	uint32_t mask = !(RCC_CFGR_MCOSEL_Msk);



	for(;;)
	{
		for(int i = 0; i<400000; i++);
		GPIOB -> ODR ^= GPIO_ODR_OD3;
	}

}
