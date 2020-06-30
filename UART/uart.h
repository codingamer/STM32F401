#ifndef __USART_H
#define __USART_H

void USART2_Init(void);
uint32_t USART2_Transmit(uint8_t * data, uint32_t len);
uint32_t USART2_Receive(uint8_t * data, uint32_t len, uint32_t timeout);
uint32_t USART2_Transmit_IT(uint8_t * data, uint32_t len);
uint32_t USART2_Receive_IT(uint8_t * data, uint32_t len, uint32_t timeout);

int __io_putchar(int ch) ;
int __io_getchar(void) ;
#endif /* __USART_H */
