#include "main.h"
#include "stm32f4xx.h"
#include "TIM2_PWM.h"

void Init_PWM(void){
	//enable GPIOA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	Set_PWM_TIM2();
	Set_PA5();
	Set_TIM2_PWM_Duty(0);
}

//This function change the duty of TIM2 PWM output duty according the percent variable
void Set_TIM2_PWM_Duty(uint32_t percent){
	TIM2->CCR1 = TIM2->ARR*percent/100;
}

//This function configure TIM2 as PWM
void Set_PWM_TIM2(void){
	//activate TIM2 Clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	//prescaler value = 10000 Hz,16 bit counter
	TIM2->PSC =(SystemCoreClock/10000) - 1;
	//autoreload register 
	TIM2->ARR = 100-1;
	//00 for the CC1S bits and 000 for OC1M bits
	TIM2->CCMR1 &= ~((3)|(7<<4));
	//setting PWM mode 1 
	TIM2->CCMR1 |= (6<<4);
	//enable CC1E bit
	TIM2->CCER |= 1;
	//setting the pulse at 0%
	TIM2->CCR1 = 0;
	//activate the PWM
	TIM2->CR1 |= TIM_CR1_CEN;
}

//This function configure the PA5 as an output open-drain with an alternate function AF01 
void Set_PA5(void){
	GPIOA->MODER &= ~((3<<5*2));
	GPIOA->MODER |= ((2<<5*2));
	GPIOA->OTYPER |= (1<<5);
	GPIOA->AFR[0] &= ~((15<<5*4));
	GPIOA->AFR[0] |= (1<<5*4);
}

}
