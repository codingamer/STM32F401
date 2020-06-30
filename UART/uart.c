#include "main.h"
#include "usart.h"
#include "stm32f4xx.h"
#include <errno.h>


//Flags to manage end of transmission and the receive
#define TX_BUSY 4
#define RX_BUSY 8

 
//device info struct
struct USART2_Device{
	uint32_t state;
	uint8_t * pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxSize;
	uint32_t RxSize;
	uint32_t TxCount;
	uint32_t RxCount;
} usart2_dev;


//initializarion of USART2 : 115200,8,1,n
void USART2_Init(void)
{
uint32_t tmp = 0, div, divmantissa, divfraction, apbclk, baud;

	//initialisation de l'USART2 : 115200,8,1,n
	baud = 115200;

	//reset/de-reset USART2
	RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
	//Enable GPIOA clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//enable USART2 clk
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	//TX on PA2 alternate function 7 (to do before setting the alternate funtion on GPIO
	GPIOA->AFR[0] &= ~(0xF << (2*4) );	//clear the 4 bits
	GPIOA->AFR[0] |= (7 << (2*4) ); 	//set alternate function Nbr 7
	//RX on PA3 alternate function 7
	GPIOA->AFR[0] &= ~(0xF << (3*4) );	//clear the 4 bits
	GPIOA->AFR[0] |= (7 << (3*4) );		//set alternate function Nbr 7
	//Configure alternate function for UART2 RX (PIN3) and TX (PIN2)
	GPIOA->MODER &= ~(3 << (2 * 2) );	//TX
	GPIOA->MODER &= ~(3 << (3 * 2) );	//RX
	GPIOA->MODER |= (2 << (2 * 2) );	//TX
	GPIOA->MODER |= (2 << (3 * 2) );	//RX
	//UART parameters configuration
	USART2->CR1 &= ~USART_CR1_UE;
	//USART CR1 Configuration
	USART2->CR1 = USART_CR1_TE | USART_CR1_RE;	//tx and rx enable; oversampling = 16
	//USART CR2 Configuration
	USART2->CR2 = 0 ; //1 stop bit
	//USART CR3 Configuration
	USART2->CR3 = 0; //no flow control
	//USART BRR Configuration
	//get APB1 prescaler
	tmp = (RCC->CFGR & RCC_CFGR_PPRE1)>>10;
	if (tmp & 4){
		tmp =  (tmp & 3) + 1;
		apbclk = SystemCoreClock >> tmp;
	}
	else {
		apbclk = SystemCoreClock;
	}
	//Tx/Rx baud = fclk /(8*(2-OVER8)*USARTDIV)
	tmp = (USART2->CR1 & USART_CR1_OVER8)>>15;
	if (tmp == 0) {
		divmantissa = (apbclk/baud) >> 4;
		divfraction = (apbclk/baud) & 0xF;
	}
	else {
		divmantissa = (apbclk/baud) >> 3;
		divfraction = (apbclk/baud) & 3;
	}
	//USART2->BRR = 0x16D for 115200 baud with fpclk= 42E6
	USART2->BRR = (divmantissa << 4) | divfraction ;
	//enable USART
	USART2->CR1 |= USART_CR1_UE;

	//interrupt
	NVIC_SetPriority(USART2_IRQn,0x5); //priority
	NVIC_EnableIRQ(USART2_IRQn);

}

int __io_putchar(int ch) 
{
	while (!(USART2->SR & USART_SR_TXE)){
	}
	USART2->DR = ch;
	return ch;
}

int __io_getchar(void)
{
		while (!(USART2->SR & USART_SR_RXNE)){}
		return (int) USART2->DR;
}


uint32_t USART2_Transmit(uint8_t * data, uint32_t len)
{
	uint32_t Nrdata = 0;

	while (Nrdata < len){
		while (!(USART2->SR & USART_SR_TXE)){}
		USART2->DR = *data++;
		Nrdata++;
	}
	//wait to the last data to be sent
	while (!(USART2->SR & USART_SR_TC)){}
	return Nrdata;
}

uint32_t USART2_Receive(uint8_t * data, uint32_t len, uint32_t timeout)
{
	uint32_t tmp, Nrdata = 0;

	while ( Nrdata < len ){
		tmp = timeout;
		//wait for RXNE to be set and decount the time-out otherwise
		while (!(USART2->SR & USART_SR_RXNE)){
			if ( (tmp--) == 0){
				if (Nrdata != 0)
					return Nrdata;
				else
					return -1 ; //no data received at all
			}
		}
		//get the datum from RDR and copy it in the application buffer
		*data++=USART2->DR ;
		//increment the data counter
		Nrdata++;
	}

	return len;
}

uint32_t USART2_Transmit_IT( uint8_t * data, uint32_t len)
{
	uint32_t ret;

	//critical section
	NVIC_DisableIRQ(USART2_IRQn);

	//test busy flag for transmission
	if (usart2_dev.state & TX_BUSY){
		//a transmission is running, quit critical section an  return an error
		NVIC_EnableIRQ(USART2_IRQn);
		return -1;
	}
	//test if there are any data to send
	if (len==0){
		NVIC_EnableIRQ(USART2_IRQn);
		return 0;
	}
	//set busy flag
	usart2_dev.state |=  TX_BUSY;
	//initialize the transmit info for ISR
	usart2_dev.pTxBuffer = data;
	usart2_dev.TxSize = len;
	usart2_dev.TxCount=0;

	//end of critical section
	NVIC_EnableIRQ(USART2_IRQn);

	//enable interrupt on transmit data register empty
	USART2->CR1 |= USART_CR1_TXEIE;

	return len;	//return immediately

}

uint32_t USART2_Receive_IT( uint8_t * data, uint32_t len, uint32_t timeout)
{
	//Critical section
	NVIC_DisableIRQ(USART2_IRQn);
	//test if the resource is free
	if( usart2_dev.state & RX_BUSY){
		errno = EAGAIN; 
		//EBUSY;
		NVIC_EnableIRQ(USART2_IRQn);
		return-1;
	}

	//simple timeout
	for(int i=0;i<timeout; i++){
		for(int j=0;j<timeout; j++){
		}
	}

	//Initialize useful variables for the interrupt
	usart2_dev.state |= RX_BUSY;
	usart2_dev.pRxBuffer = data;
	usart2_dev.RxSize = len;
	usart2_dev.RxCount=0;
	//Activate interrupt
	USART2->CR1 |= USART_CR1_RXNEIE;
	//End of critical section
	NVIC_EnableIRQ(USART2_IRQn);
	//waiting loop or immediate return
	while (usart2_dev.state & RX_BUSY) {
		__WFI();
		//note __WFE safer (if irq after the test and before __wfi) */
	}
	return usart2_dev.RxCount;
}


void USART2_IRQHandler(void)
{
	uint32_t status, cr1;

	//get status register
	status = USART2->SR;
	//get Interrupt sources
	cr1 = USART2->CR1 & 0x1F0;

	if (status & (USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE)){
		//error reception
	}

	//RXNE interrupt (a char has been received and an receive interrupt is waited upon)
	if ((status & USART_SR_RXNE) && (cr1 & USART_CR1_RXNEIE) ){
		//is there a current reception
		if (usart2_dev.state & RX_BUSY) {
			if (usart2_dev.RxCount < usart2_dev.RxSize){
				//copying data to the buffer of the application
				*usart2_dev.pRxBuffer++= USART2->DR ;
				usart2_dev.RxCount++;
				//last data copied?
				if (usart2_dev.RxCount == usart2_dev.RxSize) {
					//stop interrupt
					USART2->CR1 &=~USART_CR1_RXNEIE;
					//free the resource
					usart2_dev.state &=~RX_BUSY ;
				}
			}
		} 
	}

	//TXIE interrupt (a char has been transfered in the serializer and transmission is running)
	if ((status & USART_SR_TXE) && (cr1 & USART_CR1_TXEIE) ){
		if (usart2_dev.TxCount < usart2_dev.TxSize){
			USART2->DR = *usart2_dev.pTxBuffer++;
			usart2_dev.TxCount++;
		}
		else {
			//stop TXE interrupt
			USART2->CR1 &= ~USART_CR1_TXEIE;
			//enable TC interrupt (optional and not used here)
			USART2->CR1 |= USART_CR1_TCIE;

			//now the transmission is not busy anymore
			usart2_dev.state &= ~TX_BUSY ;

		}
	}

	//TCIE interrupt (a char has been completely transmitted)
	if ((status & USART_SR_TC) && (cr1 & USART_CR1_TCIE)){
		//stop TXE interrupt
		USART2->CR1 &= ~USART_CR1_TCIE;
		//now the transmission is not busy anymore
		usart2_dev.state &= ~TX_BUSY ;
	}

}
