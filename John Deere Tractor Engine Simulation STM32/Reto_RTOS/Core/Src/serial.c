/**
**************************
* @file : main.c
* @author : rahu7p
* @board : NUCLEO-F103RB
**************************
*
* C code to transmit the Hello World! string using the serial port (USART1)
* in bare metal
*
**************************
*/
/* Libraries, Definitions and Global Declarations */

#include <stdint.h>
#include "main.h"
#include "uart.h"
uint8_t msg[] = "Presiona la tecla 't' para encender/apagar el LED...\r\n";

/* Function prototypes */
void USER_RCC_ClockEnable( void );
void USER_GPIO_Init( void );
void USER_Delay( void );
void USER_LED_Init( void );
void USER_LED_Toggle( void );
/* Superloop structure */

/*int main(void)
{
	/* Declarations and Initializations
	USER_RCC_ClockEnable( );
	USER_GPIO_Init( );
	USER_USART1_Init( );
	USER_LED_Init();
	uint8_t t;
	/* Repetitive block
	for(;;){
		USER_USART1_Transmit(msg, sizeof(msg));
		//printf("%s",msg);
		t=USER_USART1_Receive();
		//printf("%d",t);
		if(t==0x74) {
			USER_LED_Toggle();
		}
	}
}*/
void USER_RCC_ClockEnable( void ){
	RCC->APB2ENR|= ( 0x1UL << 2U )// IO port A clock enable
				| ( 0x1UL << 14U );// USART 1 clock enable
}

void USER_GPIO_Init( void ){
//Transmit signal by the serial port
//pin PA9 (USART1_TX) as alternate function output push-pull, max speed 10MHz
	GPIOA->CRH &= ~( 0x1UL << 6U ) & ~( 0x2UL << 4U );
	GPIOA->CRH |= ( 0x2UL << 6U ) | ( 0x1UL << 4U );

//definir pa10 input floating
	GPIOA->CRH &= ~( 0x1UL << 10U ) & ~( 0x3UL << 8U );
	GPIOA->CRH |= ( 0x2UL << 10U );
	GPIOA->ODR |= ( 0x1UL << 10U); ///<<---
}

void USER_Delay( void ){
	__asm(" ldr r0, =2000 ");
	__asm(" again:sub r0, r0, #1 ");
	__asm(" cmp r0, #0 ");
	__asm(" bne again ");
}

void USER_LED_Init( void ){
	//Initialize PA5 as Output
	GPIOA->CRL = GPIOA->CRL & ~(0x3UL << 22U) & ~(0x00UL <<
	20U); //clear CNF//clear MODE
	GPIOA->CRL |= (0x1UL << 20U); //MODE(0)
}

void USER_LED_Toggle( void ){
	GPIOA->ODR ^= ( 0x1UL << 5U );
	//printf("LED Toggled"
}

