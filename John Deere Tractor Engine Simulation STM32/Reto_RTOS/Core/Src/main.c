/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include "uart.h"
#include "lcd.h"
#include <stddef.h>
#include <stdio.h>              /* This ert_main.c example uses printf/fflush */
#include "EngTrModel.h"         /* Model's header file */
#include "rtwtypes.h"
#include "keypad.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USER_B1	( GPIOC->IDR & ( 0x1UL << 13U )) //GPIOC Pin 13

//Definiciones necesarias
#define SYSCLK             64000000
#define T_HCLK             ( 1.0 / SYSCLK )
#define TIM_TIME_1S        1.0
#define TIM_PRESC_1S       ( ceil( TIM_TIME_1S / ( T_HCLK * (( 65535 + 1) - 0 ))) - 1 )
#define TIM_INIT_COUNT_1S  (( 65535 + 1 ) - ( round( TIM_TIME_1S / ( T_HCLK * ( TIM_PRESC_1S + 1 )))))

#define PERIOD_T1		200
#define PERIOD_T2		2000
#define PERIOD_T3		6000
#define PERIOD_T4		500
#define TICK_DIFF_T1	(osKernelSysTick() - (PERIOD_T1*counter++))
#define TICK_DIFF_T2	(osKernelSysTick() - (PERIOD_T2*counter++))
#define TICK_DIFF_T3	(osKernelSysTick() - (PERIOD_T3*counter++))
#define TICK_DIFF_T4	(osKernelSysTick() - (PERIOD_T4*counter++))

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId Task1Handle;
osThreadId Task2Handle;
osThreadId Task3Handle;
osThreadId Task4Handle;
osThreadId defaultTask;
osThreadId Task1;
osThreadId Task2;
osThreadId Task3;
osThreadId Task4;
osThreadId Sender1Handle;
osThreadId Sender2Handle;
osThreadId ReceiverHandle;
osMessageQId Queue1Handle;
osMessageQId Queue2Handle;
osMutexId myMutex01Handle;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask1(void const * argument);
void StartTask2(void const * argument);
void StartTask3(void const * argument);
void StartTask4(void const * argument);
void StartSender1(void const * argument);
void StartSender2(void const * argument);
void StartReceiver(void const * argument);

void Task1_Init(void);
void USER_Measure_Task1(void);
void USER_TIM2_Init();
void USER_ADC1_Enable(void);
void USER_TIM2_Init();
void turnLEDR(void);
void turnLEDL(void);
int USER_ADC1_Start(void);

//volatile float throttle = 0.0;
//char direction;
int counter = 0;

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){
	int DataIdx;
	for(DataIdx = 0; DataIdx< len; DataIdx++){
		while(!(USART2->SR & USART_SR_TXE));
		USART2 -> DR = *ptr++;
	}
	return len;
}
/* USER CODE END 0 */

void StartTask1(void const * argument)
{
  Task1_Init();//Init KEYPAD y ADC1
  osStatus status;
  osEvent adc;
  osEvent keypad1;
  uint8_t dataRX = USER_USART1_Receive();
  printf("R: %i\r\n",dataRX);
  for(;;)
  {
	osMutexWait(myMutex01Handle, osWaitForever);
	//printf("T1\r\n");
	adc = osMessageGet(Queue1Handle, 10);
	int aceleracion = adc.value.v;
	keypad1 = osMessageGet(Queue2Handle, 10);
	int keypad = keypad1.value.v;
	USER_TIM2_Delay((uint16_t)TIM_50MS_PSC, (uint16_t)TIM_50MS_CNT);
	if((keypad == 50) || (dataRX == 50)){ //50
		EngTrModel_U.Throttle = 1.45;
		EngTrModel_U.BrakeTorque = 200.0;
	}
	else if((keypad == 49) || (keypad == 51) || (dataRX == 49) || (dataRX == 51) ){ //l es 108. r es 114
		if((keypad== 49)|| (dataRX == 49)){
			turnLEDL();
		}
		else if((keypad == 51)|| (dataRX == 51)){
			turnLEDR();
		}
	}
	else if (aceleracion > 0 && keypad == 0){
		EngTrModel_U.Throttle = aceleracion;
		EngTrModel_U.BrakeTorque = 0.0;
	}
	else{
		EngTrModel_U.Throttle = 0.0;
		EngTrModel_U.BrakeTorque = 0.0;
	}
	if(adc.status == osEventMessage)
		printf("Message received: %ld\r\n", adc.value.v);
	else
		printf("Queue is empty\r\n");
	osMutexRelease(myMutex01Handle);
	osDelay(PERIOD_T1 - TICK_DIFF_T1);
	osThreadYield();
  }
}

void StartTask2(void const * argument)
{
  //Init Task 2: no hay. Ya lo incluye el modelo
  for(;;)
  {
	  //printf("T2\r\n");
	  EngTrModel_step();
	  USER_TIM2_Delay( (uint16_t)TIM_700MS_PSC, (uint16_t)TIM_700MS_CNT);//  500ms
	  osDelay(PERIOD_T2 - TICK_DIFF_T2);
	  osThreadYield();
  }
}

void StartTask3(void const * argument)
{
  LCD_Init( );//Init Task 3
  osStatus status;
  osEvent adc;
  for(;;)
    {
	 osMutexWait(myMutex01Handle, osWaitForever);
  	//printf("T1\r\n");
    adc = osMessageGet(Queue1Handle, 10);
  	int aceleracion = adc.value.v;
	 LCD_Clear();
	 LCD_Set_Cursor(1, 1);
	 LCD_Put_Str("AC:");
	 LCD_Put_Num((int)aceleracion);
	 if((int)aceleracion == 0){
		 LCD_Put_Str("0");
	 }
	 LCD_Set_Cursor(1,8);
	 LCD_Put_Str("VS:");
	 LCD_Put_Num((int)EngTrModel_Y.VehicleSpeed);
	 LCD_Put_Str("ms");
	 if((int)EngTrModel_Y.VehicleSpeed == 0){
		 LCD_Put_Str("0");
	 }
	 LCD_Set_Cursor(2,1);
	 LCD_Put_Str("ES:");
	 if (EngTrModel_Y.EngineSpeed < 0) {
		 LCD_Put_Str("-");
		 LCD_Put_Num((int)-EngTrModel_Y.EngineSpeed);
		 LCD_Put_Str("rpm");
	 } else {
		 LCD_Put_Num((int)EngTrModel_Y.EngineSpeed);
		 LCD_Put_Str("rpm");
	 }
	 LCD_Set_Cursor(2,12);
	 LCD_Put_Str("G:");
	 LCD_Put_Num((int)EngTrModel_Y.Gear);

	 USER_TIM2_Delay( (uint16_t)TIM_700MS_PSC, (uint16_t)TIM_700MS_CNT);//  500ms
	 osMutexRelease(myMutex01Handle);
	 osDelay(PERIOD_T3 - TICK_DIFF_T3);
	 osThreadYield();
  }
}

void StartTask4(void const * argument)
{
  USER_USART1_Init(); //Init Task 4
  for(;;)
  {
	  //printf("T4\r\n");
	  printf("%i,%i,%i,%i\n\r", (int)EngTrModel_Y.VehicleSpeed, (int)EngTrModel_Y.EngineSpeed, (int)EngTrModel_Y.Gear, USER_ADC1_Start());
	  osDelay(PERIOD_T4 - TICK_DIFF_T4);
	  osThreadYield();
  }
}

void StartSender1(void const * argument){
	osStatus status;
	uint32_t aceleracion = USER_ADC1_Start();
	for(;;){
		status = osMessagePut(Queue1Handle, aceleracion, 0);
		osDelay(1000);
	    osThreadYield();

	}
}

void StartSender2(void const * argument){
	osStatus status;
	uint8_t keypad = KEYPAD_ReadKey();
	for(;;){
		status = osMessagePut(Queue2Handle, keypad, 0);
		osDelay(1000);
	    osThreadYield();
	}
}

void StartReceiver(void const * argument){
	uint32_t counter = 0;
	osStatus status;
	osEvent keypad;
	for(;;)
	{
	/* Get item from Queue with a 0 timeout */
		keypad = osMessageGet(Queue2Handle, 10);
		if(keypad.status == osEventMessage)
			printf("Msg: %ld\r\n", keypad.value.v);
		else
			printf("Queue is empty\r\n");
	osDelay(999 - (osKernelSysTick() - (999 * counter++)));
	}
}

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();

  /* USER CODE BEGIN Init */
  //Task1_Init();//Init KEYPAD y ADC1
  	//Init Task 2: no hay. Ya lo incluye el modelo
  //LCD_Init( );//Init Task 3
  //USER_USART1_Init(); //Init Task 4
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
 // osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /*osThreadDef(Receiver, StartReceiver, osPriorityNormal, 0, 128);
  ReceiverHandle = osThreadCreate(osThread(Receiver), NULL);*/

  osThreadDef(Task2, StartTask2, osPriorityNormal, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  osThreadDef(Task4, StartTask4, osPriorityAboveNormal, 0, 128);
  Task4Handle = osThreadCreate(osThread(Task4), NULL);

  osThreadDef(Task3, StartTask3, osPriorityBelowNormal, 0, 128);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  osThreadDef(Task1, StartTask1, osPriorityHigh, 0, 128);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  osThreadDef(Sender1, StartSender1, osPriorityNormal, 0, 128);
  Sender1Handle = osThreadCreate(osThread(Sender1), NULL);

  osThreadDef(Sender2, StartSender2, osPriorityNormal, 0, 128);
  Sender2Handle = osThreadCreate(osThread(Sender2), NULL);

  osMessageQDef(Queue1, 1, uint32_t);
  Queue1Handle = osMessageCreate(osMessageQ(Queue1), NULL);

  osMessageQDef(Queue2, 1, uint8_t);
  Queue2Handle = osMessageCreate(osMessageQ(Queue2), NULL);

  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	printf("IT\r\n");
    HAL_Delay(1000);
  }
  /* USER CODE END 5 */
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void Task1_Init(){
	USER_TIM2_Init();
	KEYPAD_Init();
  	USER_ADC1_Enable();
}

void USER_TIM2_Init(){
	//USER_RCC_Init();
	/* System Clock configuration for 64 Mhz */
	FLASH->ACR	&= ~( 0x5UL <<  0U );	//Two wait states latency, if SYSCLK > 48Mhz
	FLASH->ACR 	|=  ( 0x2UL <<  0U );	//Two wait states latency, if SYSCLK > 48Mhz

	RCC->CFGR	&= ~( 0x1UL << 16U )
				&  ~( 0x7UL << 11U )
				&  ~( 0x3UL <<  8U )
				&  ~( 0xFUL <<  4U );

	RCC->CFGR 	|= ( 0xFUL << 18U )
				|  ( 0x4UL <<  8U );
	RCC->CR		|= ( 0x1UL << 24U );
	while ( !(RCC->CR & ~( 0x1UL << 25U )));
	RCC->CFGR 	&= ~( 0x1UL << 0U );
	RCC->CFGR	|=  ( 0x2UL << 0U );
	while ( 0x8UL != ( RCC->CFGR & 0xCUL ));

	//USER_RCC_ClockEnable();
	//TIM2 Clock Enable
	RCC->APB1ENR |= ( 0x1UL << 0U );

	//USER_GPIO_Init()
	//TIMER INITIALIZATION/
	// Timer slave mode disabled
	TIM2->SMCR &= ~(0x1 << 0U)
			  &	 ~(0x1 << 1U)
			  &	 ~(0x1 << 2U);

	// Configure the counter mode, Upcounter and the overflow UEV-event
	TIM2->CR1 &= ~(0x1 << 6U)
			  &	 ~(0x1 << 5U)
			  &	 ~(0x1 << 4U)
			  &	 ~(0x1 << 1U);

	//Enable internal clock
	//TIM2->SMCR &= ~( 0x7UL << 0U );
	//UEV enabled
	TIM2->CR1  &= ~( 0x1UL << 1U );
	//Upcounter
	TIM2->CR1  &= ~( 0x1UL << 4U );
	//Edge-alingned mode
	TIM2->CR1  &= ~( 0x3UL << 5U );
}

void USER_ADC1_Enable(void){
  	//USER_RCC_ClockEnable();
	//IO port A clock enable
	RCC->APB2ENR |= ( 0x1UL << 2U );
	//ADC Clock Enable
	RCC->APB2ENR |= ( 0x1UL << 9U );
	//Adjust ADC input clock
	RCC->CFGR 	 |= ( 0x3UL << 14U);	// Prescaler 8

	//USER_GPIO_Init();
	//PA0 as analog input for potentiometer
	GPIOA->CRL &= ~( 0xFUL << 0 );

	//USER_ADC1_Init();
	//Select Independent Mode
	ADC1->CR1 	&= ~( 0xFUL << 16U );
	//Right Alignment and Continuous Conversion Mode
	ADC1->CR2 	|=  ( 0x1UL <<  1U );
	ADC1->CR2 	&= ~( 0x1UL << 11U );
	//Sample time of 1.5 cycles for Channel 0
	ADC1->SMPR2 &=  ~( 0x7UL << 0U );
	//1 Conversion for Regular Channels
	ADC1->SQR1	&= ~( 0xFUL << 20U );
	//Select Channel 0 for First Conversion
	ADC1->SQR3  &= ~( 0x1FUL << 0U );
	//Enable ADC Module
	ADC1->CR2	|=  ( 0x1UL << 0U );
	//Module Calibration
	ADC1->CR2 	|=  ( 0x1UL << 2U ); // Start calibration
	//Wait until calibration is complete
	while ((ADC1->CR2 & ( 0x1UL << 2U )));
}

int USER_ADC1_Start(void){
	//Conversion starts when this bit holds a value of 1 and a 1 is written to it
	ADC1->CR2	|=  ( 0x1UL << 0U );
	//Wait until conversion is done
	while (!(ADC1->SR & ( 0x1UL << 1U )));
	//Read value from
	uint16_t adc_value = ADC1->DR;
	//Reset conversion flag
	ADC1->SR &= ~( 0x1UL << 1U );
	//Return value to percentage
	return (adc_value / 4095.0) * 100;
}

void USER_TIM2_Delay(uint16_t prescaler, uint16_t counter){
	//Clear the timer UIF
	TIM2->SR   &= ~( 0x1UL << 0U );
	//Configure initial count and prescaler
	TIM2->CNT 	= counter;
	TIM2->PSC 	= prescaler;
	//Enable timer to start counting
	TIM2->CR1  |= ( 0x1UL << 0U );
	//Wait for overflow
	while (! ( TIM2->SR & ( 0x1UL << 0U ) ));
	//Stop timer
	TIM2->CR1  &= ~( 0x1UL << 0U );
}

// 50 ms delay
void USER_TIM_Delay3(uint32_t time) {
	// Set the CNT and PSC values
	time = time * 1000000;
	TIM2->PSC = ceil(time/(3.125*65536))-1;
	TIM2->CNT = 65536-(time/((TIM2->PSC+1)*3.125));

	// Clear the timer interrupt flag
	TIM2->SR  &= ~(0x1 << 0U);

	// Initialize Timer
	TIM2->CR1 |= (0x1 << 0U);

	while (!(TIM2->SR & (0x1UL << 0U)));

	// Initialize Timer
	TIM2->CR1 &= ~(0x1 << 0U);

}

//PB15 LEDR
void turnLEDR(void){
	GPIOB->BSRR	=	( 0x1UL <<  15U );//	value to set pin7 of port C
	USER_TIM2_Delay( (uint16_t)TIM_50MS_PSC, (uint16_t)TIM_50MS_CNT);
	//Buzzer PA7
	GPIOA->BSRR	=	( 0x1UL <<  7U );//	value to set pin7 of port A
	USER_TIM2_Delay( (uint16_t)TIM_50MS_PSC, (uint16_t)TIM_50MS_CNT);
	GPIOB->BSRR	=	( 0x1UL << 31U );//	value to reset pin7 of port C
	USER_TIM2_Delay( (uint16_t)TIM_50MS_PSC, (uint16_t)TIM_50MS_CNT);
	//Buzzer
	GPIOA->BSRR	=	( 0x1UL <<  23U );//	value to set pin7 of port A
	USER_TIM2_Delay( (uint16_t)TIM_50MS_PSC, (uint16_t)TIM_50MS_CNT);
}

//PB1 LEDL
void turnLEDL(void){
	GPIOB->BSRR	=	( 0x1UL <<  1U );//	value to set pin8 of port A
	USER_TIM2_Delay( (uint16_t)TIM_50MS_PSC, (uint16_t)TIM_50MS_CNT);
	//Buzzer PA7
	GPIOA->BSRR	=	( 0x1UL <<  7U );//	value to set pin7 of port A
	USER_TIM2_Delay( (uint16_t)TIM_50MS_PSC, (uint16_t)TIM_50MS_CNT);
	GPIOB->BSRR	=	( 0x1UL << 17U );//	value to reset pin8 of port A
	USER_TIM2_Delay( (uint16_t)TIM_50MS_PSC, (uint16_t)TIM_50MS_CNT);
	//Buzzer
	GPIOA->BSRR	=	( 0x1UL <<  23U );//	value to set pin7 of port A
	USER_TIM2_Delay( (uint16_t)TIM_50MS_PSC, (uint16_t)TIM_50MS_CNT);
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
