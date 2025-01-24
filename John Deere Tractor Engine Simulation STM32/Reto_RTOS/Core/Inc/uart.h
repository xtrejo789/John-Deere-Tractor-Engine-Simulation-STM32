#ifndef UART_H_
#define UART_H_

#include <stdio.h>
#define USARTDIV 0x22C// 115200 baud rate
/*#define USART_CR1_UE ( 0x1UL << 13U )
#define USART_CR1_M ( 0x1UL << 12U )
#define USART_CR1_TE ( 0x1UL << 3U )
#define USART_CR1_RE ( 0x1UL << 2U )
#define USART_CR2_STOP ( 0x3UL << 12U )
#define USART_SR_TXE ( 0x1UL << 7U )
#define USART_SR_RXNE ( 0x1UL << 5U ) //Transmit data */

void USER_USART1_Init( void );
void USER_USART1_Transmit( uint8_t *pData, uint16_t size );
uint8_t USER_USART1_Receive( void );
int _write(int file, char *ptr, int len);
#endif /* UART_H_ */
