#include "main.h"
#include "stm32f4xx.h"
#include "stdlib.h"
#include "TIM3_CAPTURE_MODE.h"

static volatile uint32_t TIM3_pulse=0;
static volatile uint32_t TIM3_overflow=0;

void Init_Capture_Mode(int flag){
	//enable GPIOB clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	Set_PB0();
	Set_Capture_Mode_TIM3(flag);
}

//This function configure TIM3 as capture mode with enabling interruption
void Set_Capture_Mode_TIM3(int flag){
	//activate TIM3 Clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	//prescaler value = 10000 Hz,16 bit counter
	TIM3->PSC =(SystemCoreClock/10000) - 1; //SystemCoreClock=84 MHz
	//autoreload register 
	TIM3->ARR = 0xFFFF;
	//In TIM3_CCER
		//Put CC3E and CC4E bits at 0
		TIM3->CCER &= ~((1<<8)|(1<<12));
	//In TIM3_CCMR2
		//Put IC3F at 0010
		TIM3->CCMR2 &= ~((15<<4));
		TIM3->CCMR2 |= (2<<4);
		//Put IC3PSC and IC4PSC at 0
		TIM3->CCMR2 &= ~((3<<2)|(3<<10));
		//Put CC3S at 01 and CC4S at 10
		TIM3->CCMR2 &= ~((3<<0)|(3<<8));
		TIM3->CCMR2 |= ((1<<0)|(2<<8));
	//In TIM3_CCER
		//Put CC3NP/CC3P at 01, CC4NP/CC4P at 00, CC3E and CC4E at 1
		TIM3->CCER &= ~((1<<15)|(1<<13)|(1<<11)|(1<<9));
		TIM3->CCER |= ((1<<9)|(1<<8)|(1<<12));
	//In TIM3_DIER
		//Put UIE and CC4IE at 1
		TIM3->DIER &= ~(3<<3);
		TIM3->DIER |= ((2<<3)|(1<<0));
	if (flag){
		//enable inetrruption for TIM3
		NVIC->ISER[0] |= (1 << 29);
		NVIC->IP[29] |= (5 << 4);	
	}
	//enable TIM3
	TIM3->CR1 |= TIM_CR1_CEN;
}

//This function configure PB0 as input pull-up with alternate function AF02
void Set_PB0(void){
	GPIOB->MODER &= ~((3<<0*2));
	GPIOB->MODER |= (2<<0*2);
	GPIOB->PUPDR &= ~((3<<0*2));
	GPIOB->PUPDR |= (1<<0*2);
	GPIOB->AFR[0] &= ~((15<<0*4));
	GPIOB->AFR[0] |= (2<<0*4); 
 	//if (~(PB->IDR) & (1<<0)){} to detect the entry
}

int get_val(void){
	return abs(TIM3_pulse);
}

/* TIM3 IRQ Handler  */
void TIM3_IRQHandler(void){
	uint32_t pulse;
	TIM_TypeDef* tim = TIM3;
	//test uif for overflow
	if(tim->SR & (1<<0)){
		//overflow of the main counter
		//if a measure has started (CCR3 has captured a value but not CCR4 * count overflow)
		if (tim->SR & (1<<3))
			TIM3_overflow += 1;
		//ack the update event
		tim->SR &= ~(1<<0);
	}
	if (tim->SR & (1<<4)){
		//capture in CCR4
		pulse=tim->CCR4+TIM3_overflow*(tim->ARR+1)-tim->CCR3;
		//pulse in millisecond
		pulse/=10;
		//reset overflow
		TIM3_overflow=0;
		//ack
		tim->SR &=~((1<<12)|(1<<4)|(1<<11)|(1<<3));
		//update pulse width
		TIM3_pulse=pulse;
	}

}
