#include "stm32f4xx.h"
#include "main.h"
#include "TIM5.h"

//This function configre TIM5 to generate an interruption each Nms
void TIM5_Config(uint32_t N){
	//activate TIM5 Clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	//prescaler value = 2000 Hz,16 bit counter
	TIM5->PSC = (SystemCoreClock / 1000 / 2 ) - 1;
	//autoreload register 
	TIM5->ARR = (uint32_t) (N*2-1);
	//clear counter and prescaler (update forced)
	TIM5->EGR = TIM_EGR_UG;
	//Clear interrupt flags in status register
	TIM5->SR = 0 ;
	//configure interrupt
	TIM5->DIER |= TIM_DIER_UIE;
	NVIC->ISER[1] |= (1 << (50-32));
	NVIC->IP[50] |= (7 << 4);
	//enable the counter
	TIM5->CR1 |= TIM_CR1_CEN;
}

//TIM5 Handler
void TIM5_IRQHandler(void){
	if((TIM5->SR & TIM_SR_UIF) && (TIM5->DIER & TIM_DIER_UIE)){
		/*
		****************
		****************
		**DO SOMETHING**
		****************
		****************
		*/
		TIM5->SR &= ~(TIM_SR_UIF);	
	}
}


