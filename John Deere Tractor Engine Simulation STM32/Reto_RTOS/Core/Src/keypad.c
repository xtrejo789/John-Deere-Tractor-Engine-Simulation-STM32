#include "main.h"
#include "keypad.h"

// Matriz de teclas con cuatro filas y tres columnas
static const char KEYPAD_MAP[4][3] = {
    {'1', '2', '3'},  // Primera fila
    {'4', '5', '6'},  // Segunda fila
    {'7', '8', '9'},  // Tercera fila
    {'*', '0', '#'}   // Cuarta fila
};

void KEYPAD_Init(void)
{
	//CLOCKENABLE
	//IO port A clock enable
	RCC->APB2ENR |= ( 0x1UL << 2U );

	//CLOCKENABLE
	//IO port B clock enable
	RCC->APB2ENR |= ( 0x1UL << 3U );

	//LED Der PB15

	// GPIOx_BSRR modified to reset pin7 of port C
	GPIOB->BSRR	=	( 0x1UL << 31U );//	immediate value

	// GPIOx_CRH modified to configure pin7 as output
	GPIOB->CRH	=	GPIOB->CRH//		GPIOx_CRH actual value
				&//			to clear
				~( 0x3UL << 30U )//	(mask) CNF7[1:0] bits
				&//			to clear
				~( 0x2UL << 28U );//	(mask) MODE5_1 bit

	// GPIOx_CRL modified to select pin7 max speed of 10MHz
	GPIOB->CRH	=	GPIOB->CRH//		GPIOx_CRH actual value
				|//			to set
				( 0x1UL << 28U );//	(mask) MODE8_0 bit

	//LED Izq PB1

	// GPIOx_BSRR modified to reset pin4 of port B
	GPIOB->BSRR	=	( 0x1UL << 17U );//	immediate value

	// GPIOx_CRH modified to configure pin4 as output
	GPIOB->CRL	=	GPIOB->CRL//		GPIOx_CRH actual value
				&//			to clear
				~( 0x3UL << 6U )//	(mask) CNF8[1:0] bits
				&//			to clear
				~( 0x2UL << 4U );//	(mask) MODE5_1 bit

	// GPIOx_CRL modified to select pin4 max speed of 10MHz
	GPIOB->CRL	=	GPIOB->CRL//		GPIOx_CRH actual value
				|//			to set
				( 0x1UL << 4U );//	(mask) MODE5_0 bit

	//Buzzer PA7

	// GPIOx_BSRR modified to reset pin7 of port A
	GPIOA->BSRR	=	( 0x1UL << 23U );//	immediate value

	// GPIOx_CRH modified to configure pin8 as output
	GPIOA->CRH	=	GPIOA->CRH//		GPIOx_CRH actual value
				&//			to clear
				~( 0x3UL << 30U )//	(mask) CNF8[1:0] bits
				&//			to clear
				~( 0x2UL << 28U );//	(mask) MODE5_1 bit

	// GPIOx_CRL modified to select pin8 max speed of 10MHz
	GPIOA->CRH	=	GPIOA->CRH//		GPIOx_CRH actual value
				|//			to set
				( 0x1UL << 28U );//	(mask) MODE5_0 bit


    // Configura PA5 (fila 1) como entrada con resistencia pull-down
    GPIOA->CRL &= ~(0xFUL << 20U);  // Limpia la configuración anterior de PA5
    GPIOA->CRL |= (0x8UL << 20U);   // Entrada con pull-down

    // Configura PA8 (fila 2) como entrada con resistencia pull-down
    GPIOA->CRH &= ~(0xFUL << 0U);  // Limpia la configuración anterior de PA8
    GPIOA->CRH |= (0x8UL << 0U);   // Entrada con pull-down

    // Configura PA1 (fila 3) como entrada con resistencia pull-down
    GPIOA->CRL &= ~(0xFUL << 4U);  // Limpia la configuración anterior de PA1
    GPIOA->CRL |= (0x8UL << 4U);   // Entrada con pull-down

    // Configura PA4 (fila 4) como entrada con resistencia pull-down
    GPIOA->CRL &= ~(0xFUL << 16U);  // Limpia la configuración anterior de PA4
    GPIOA->CRL |= (0x8UL << 16U);   // Entrada con pull-down

    // Configura PA6 (columna 1) como salida push-pull
    GPIOA->CRL &= ~(0xFUL << 24U);  // Limpia la configuración anterior de PA6
    GPIOA->CRL |= (0x1UL << 24U);   // Salida push-pull

    // Configura PA7 (columna 2) como salida push-pull
    GPIOA->CRL &= ~(0xFUL << 28U);  // Limpia la configuración anterior de PA7
    GPIOA->CRL |= (0x1UL << 28U);   // Salida push-pull

    // Configura PA10 (columna 3) como salida push-pull
    GPIOA->CRH &= ~(0xFUL << 8U);  // Limpia la configuración anterior de PA10
    GPIOA->CRH |= (0x1UL << 8U);   // Salida push-pull

    // Inicializa todas las columnas (PA6, PA7 y PA10) en estado LOW (desactivadas)
    GPIOA->ODR &= ~(0x1UL << 6);
    GPIOA->ODR &= ~(0x1UL << 7);
    GPIOA->ODR &= ~(0x1UL << 10);
}

char KEYPAD_ReadKey(void)
{
    char key = '\0';  // Inicializa sin carácter presionado

    // Activa PA6 (columna 1) en HIGH
    GPIOA->ODR |= (0x1UL << 6);

    // Verifica cada fila (PA5, PA8, PA1, PA4)
    if (GPIOA->IDR & (0x1UL << 5))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[0][0];  // Retorna '1'
    }
    else if (GPIOA->IDR & (0x1UL << 8))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[1][0];  // Retorna '4'
    }
    else if (GPIOA->IDR & (0x1UL << 1))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[2][0];  // Retorna '7'
    }
    else if (GPIOA->IDR & (0x1UL << 4))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[3][0];  // Retorna 'A'
    }

    // Desactiva PA6 (columna 1) volviéndola a LOW
    GPIOA->ODR &= ~(0x1UL << 6);

    // Activa PA7 (columna 2) en HIGH
    GPIOA->ODR |= (0x1UL << 7);

    // Verifica cada fila (PA5, PA8, PA1, PA4)
    if (GPIOA->IDR & (0x1UL << 5))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[0][1];  // Retorna '2'
    }
    else if (GPIOA->IDR & (0x1UL << 8))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[1][1];  // Retorna '5'
    }
    else if (GPIOA->IDR & (0x1UL << 1))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[2][1];  // Retorna '8'
    }
    else if (GPIOA->IDR & (0x1UL << 4))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[3][1];  // Retorna 'B'
    }

    // Desactiva PA7 (columna 2) volviéndola a LOW
    GPIOA->ODR &= ~(0x1UL << 7);

    // Activa PA10 (columna 3) en HIGH
    GPIOA->ODR |= (0x1UL << 10);

    // Verifica cada fila (PA5, PA8, PA1, PA4)
    if (GPIOA->IDR & (0x1UL << 5))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[0][2];  // Retorna '3'
    }
    else if (GPIOA->IDR & (0x1UL << 8))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[1][2];  // Retorna '6'
    }
    else if (GPIOA->IDR & (0x1UL << 1))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[2][2];  // Retorna '9'
    }
    else if (GPIOA->IDR & (0x1UL << 4))
    {
        for (volatile int i = 0; i < 10000; i++);
        key = KEYPAD_MAP[3][2];  // Retorna 'C'
    }

    // Desactiva PA10 (columna 3) volviéndola a LOW
    GPIOA->ODR &= ~(0x1UL << 10);

    return key;
}
