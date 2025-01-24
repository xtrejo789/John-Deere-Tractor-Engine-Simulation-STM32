/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void USER_TIM2_Delay(uint16_t prescaler, uint16_t counter);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TIM_10MS_PSC 9		//Prescaler value for 10ms
#define TIM_10MS_CNT 1536	//Counter value for 10ms

#define TIM_200MS_PSC 195	//Prescaler value for 200ms
#define TIM_200MS_CNT 229	//Counter value for 200ms

#define TIM_50MS_PSC 48	//Prescaler value for 50ms
#define TIM_50MS_CNT 229	//Counter value for 50ms

#define TIM_5MS_PSC 4	//Prescaler value for 5ms
#define TIM_5MS_CNT 1536	//Counter value for 5ms

#define TIM_1MS_PSC 0	//Prescaler value for 1ms
#define TIM_1MS_CNT 1536	//Counter value for 1ms

#define TIM_700MS_PSC 683	//Prescaler value for 700ms
#define TIM_700MS_CNT 39	//Counter value for 700ms

#define TIM_1000MS_PSC 976	//Prescaler value for 1000ms
#define TIM_1000MS_CNT 30	//Counter value for 700ms

#define TIM_500MS_PSC 488	//Prescaler value for 500ms
#define TIM_500MS_CNT 96	//Counter value for 500ms

#define TIM_100US_PSC 0		//Prescaler value for 100us
#define TIM_100US_CNT 59136	//Counter value for 100us

#define TIM_10US_PSC 0	//Prescaler value for 100us
#define TIM_10US_CNT 64896	//Counter value for 100us

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
