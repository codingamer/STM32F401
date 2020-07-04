#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx.h"

#include "main.h"
#include "usart.h"
#include "GENERATE_PULSE.h"

#include "FreeRTOS.h"

#include "task.h"
#include "list.h"
#include "queue.h"
#include "portmacro.h"
#include "semphr.h"

#define STACK_SIZE	1024 // in words (= 4*STACK_SIZE bytes)

static SemaphoreHandle_t stdio_mutex=NULL;
static SemaphoreHandle_t xSemph_sleep=NULL;

static void SystemClock_Config(void);
void wait_ms(uint32_t);
void Config_EXTI_PC13(void);

void Deep_Sleep_Tsk(void *);
void Simple_Task(void *);


//approximative active waiting function
void  __attribute__((optimize("O1"))) wait_ms(uint32_t ms){
	ms =(SystemCoreClock/1000/4)*ms;
	while(ms--)
		asm("");	// avoid the function to be optimized out !
}


int main(void)
{
	//creating mutex to protect the serial printing
	stdio_mutex = xSemaphoreCreateMutex();

	//creating binary semaphore			
	xSemph_sleep = xSemaphoreCreateBinary();

	TaskHandle_t xHandle = NULL;
	BaseType_t xReturned;

	/* Configure the System clock to 84 MHz */
	SystemClock_Config();

	/*priority grouping 4 bits for pre-emption priority 0 bits for subpriority (No Subpriority) for FreeRTOS*/
	NVIC_SetPriorityGrouping(3);

	/* init the USART2 */
	USART2_Init();

	/* init PA6 */
	Init_Pulse();
	GPIOA->ODR &= ~(1<<6);

	/* init PC13 */
	Config_EXTI_PC13();

//	setvbuf( stdout, NULL , _IOLBF, (size_t) 10);


	//Create the tasks, storing the handle
	xReturned = xTaskCreate(Simple_Task,"Simple_Task",STACK_SIZE,( void * ) 0,tskIDLE_PRIORITY+1,&xHandle);
	if( xReturned == pdPASS )
	{
		//The task was created.  Use the task's handle to delete the task
		//vTaskDelete( xHandle );
	}else{
		//not enough memory to create task
		printf(" not enough memory to create task Simple_Task \r\n");
	}

	xReturned = xTaskCreate(Deep_Sleep_Tsk,"Deep_Sleep_Tsk",STACK_SIZE,( void * ) 0,tskIDLE_PRIORITY+2,&xHandle);
	if( xReturned == pdPASS )
	{
		//The task was created.  Use the task's handle to delete the task
		//vTaskDelete( xHandle );
	}else{
		//not enough memory to create task
		printf(" not enough memory to create task Deep_Sleep_Tsk \r\n");
	}

	printf("Start of application \r\n");

	vTaskStartScheduler();

	for(;;){
	}

	return 0;
}

void Deep_Sleep_Tsk(void *pvParameters){
	while(1){
		//waiting to pc13 push button
		xSemaphoreTake(xSemph_sleep,portMAX_DELAY);
		USART2_Transmit("Sleep ...\n\r",11);
		//deactivate all interrupt
		__disable_irq();
		//activate the deep sleep bit of the cortex-M
		SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP_Pos);
		//activate the stop mode of the STM32
		PWR->CR |= PWR_CR_LPDS_Msk;
		//disable systick		
		SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
		//sleep
		__WFI();
		//enable systick
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
		//disable the deep sleep
		SCB->SCR &= ~(1 << SCB_SCR_SLEEPDEEP_Pos);
		//change back the SoC main clock that is now the HSI RC (internal RC oscillator)
		SystemClock_Config();
		//activate the interrupt bac
		__enable_irq();
		USART2_Transmit("Soc is awaken\n\r",15);
		//consuming xSemph_sleep of awaking
		xSemaphoreTake(xSemph_sleep,portMAX_DELAY);
		
	}
}
 
void Simple_Task(void *pvParameters){
	char data[50];
	for (int i=0;i<50;i++){
		data[i]='\000';
	}
	sprintf(data,"This is a message from Simple_Task\0");
	while (1){
		xSemaphoreTake(stdio_mutex,portMAX_DELAY);
		USART2_Transmit(data,50);
		USART2_Transmit("\n\r",2);
		xSemaphoreGive(stdio_mutex);
		vTaskDelay(pdMS_TO_TICKS(1000));// 1s sleep
	}
}

void vApplicationIdleHook(void){
	/* simple sleep, only the core is stopped, 
	peripheral continue to work * the core will be woken 
	at next tick or any other interrupt */
	if (xSemaphoreTake(stdio_mutex,0)){
		printf("get tick : %lu \r\n", (uint32_t) xTaskGetTickCount());
		xSemaphoreGive(stdio_mutex);
	}
}


void Config_EXTI_PC13(void){
	//Enable the clock for GPIOC
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	//Enable the clock for SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	//Configure PC13 as input
	GPIOC->MODER &= ~(3<<(2*13));
	//Select PC13 to drive the EXTI13 input of the EXTI
	SYSCFG->EXTICR[3] &= ~(0xF<<4);
	SYSCFG->EXTICR[3] |= (0x2<<4);
	//Configure the EXTI to trigger an interrupt on the falling edge of the pin and to enable the interrup
	EXTI->FTSR |= (1<<13); 
	EXTI->IMR |= (1<<13);
	//Set the interrupt priority for EXTI15_10 and enable it 
	NVIC_SetPriority (EXTI15_10_IRQn, (configMAX_SYSCALL_INTERRUPT_PRIORITY>>4)+1);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	//The GPIO and SYSCFG can be now be deactivated, the EXTI clock stay activated
	RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
}

void  EXTI15_10_IRQHandler(void){
	BaseType_t param;
	param=pdFALSE;
	if ((EXTI->IMR & (1<<13)) && (EXTI->PR & (1<<13))){
		GPIOA->ODR ^= (1<<6);
		//Interrupt Acknowledge
		EXTI->PR |= (1<<13);
		xSemaphoreGiveFromISR(xSemph_sleep, &param);
	} 
}

static void SystemClock_Config()
{

	/* increase the flash WS first (1WS for f > 30 and 2WS for f > 60MHz*/
		while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2WS)
	   		FLASH->ACR = (FLASH->ACR & (~FLASH_ACR_LATENCY)) | FLASH_ACR_LATENCY_2WS;

	/* acceleration cache I & D (literal pool) + prefetch to compensate for the 2 wait states */
		FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_DCEN;


	RCC->CR |= RCC_CR_HSEBYP; /* external oscillator */
    	/* enable clock using HSE instead of HSI*/
	    	RCC->CR |= RCC_CR_HSEON  | RCC_CR_HSEBYP ;
	/* wait for HSE to be stable */
		while ((RCC->CR & RCC_CR_HSERDY) == 0) ;

	RCC->CR &= ~RCC_CR_PLLON;
	/* PLL configuration */
		PWR->CR |= PWR_CR_VOS_1;
		/* PLL configuration
		*	PLL_M                          = 8
		*	PLL_N                          = 336
		*	PLL_P                          = 4
		*	PLL_Q                          = 7
		*/
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ );
		RCC->PLLCFGR |=  (RCC_PLLCFGR_PLLM & (8<<0))
		                | (RCC_PLLCFGR_PLLN & (336<<6))
		                | (RCC_PLLCFGR_PLLP & (1 << 16))
		                | (RCC_PLLCFGR_PLLQ & (7<<24));
		RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE ;

	/* PLL enable */
		RCC->CR |= RCC_CR_PLLON;
	/* wait for the PLL to be stable */
		while ((RCC->CR & RCC_CR_PLLRDY) == 0) ;


	/* modify CPU clock sources */
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){
			RCC->CFGR &= ~RCC_CFGR_SW;
			RCC->CFGR |= RCC_CFGR_SW_PLL ;//RCC_CFGR_SW_HSI; RCC_CFGR_SW_HSE;
		}
	/* switch off HCI */
		RCC->CR &= ~RCC_CR_HSION;
	/* modify AHB divisor (no DIV)*/
		RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_HPRE))| RCC_CFGR_HPRE_DIV1 ;
	/* Modify APB1 (low speed) divisor DIV2 */
		RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE1))| RCC_CFGR_PPRE1_DIV2 ;
	/* Modify APB2 (high speed) no div */
		RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE2))| RCC_CFGR_PPRE2_DIV1 ;

	SystemCoreClockUpdate(); //SystemCoreClock = 84000000; ((HSE_VALUE / PLLM) / PLLP) * PLLN / HPRE_DIVN
}


#ifdef  USE_FULL_ASSERT	// defined in makefile
void assert_failed(char* file, uint32_t line)
{
	while (1)
	{
	}
}
#endif
