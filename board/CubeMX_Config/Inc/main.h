/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define USING_RTOS
#define USING_DMA
#define CUSTOM_MALLOC rt_malloc
#define CUSTOM_FREE rt_free
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//extern void _Error_Handler(char *s, int num);
//#ifndef Error_Handler
//#define Error_Handler() 
#define __SET_FLAG(__OBJECT, __BIT) (((__OBJECT) |= 1U << (__BIT)))
#define __RESET_FLAG(__OBJECT, __BIT) (((__OBJECT) &= ~(1U << (__BIT))))
#define __GET_FLAG(__OBJECT, __BIT) (((__OBJECT) & (1U << (__BIT))))
//#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
//#endif
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WIFI_RESET_Pin GPIO_PIN_12
#define WIFI_RESET_GPIO_Port GPIOE
#define WIFI_LINK_Pin GPIO_PIN_13
#define WIFI_LINK_GPIO_Port GPIOE
#define WIFI_RELOAD_Pin GPIO_PIN_14
#define WIFI_RELOAD_GPIO_Port GPIOE
#define WIIF_READY_Pin GPIO_PIN_15
#define WIIF_READY_GPIO_Port GPIOE
#define DI_7_Pin GPIO_PIN_11
#define DI_7_GPIO_Port GPIOD
#define DI_6_Pin GPIO_PIN_12
#define DI_6_GPIO_Port GPIOD
#define DI_5_Pin GPIO_PIN_13
#define DI_5_GPIO_Port GPIOD
#define DI_4_Pin GPIO_PIN_14
#define DI_4_GPIO_Port GPIOD
#define DI_3_Pin GPIO_PIN_15
#define DI_3_GPIO_Port GPIOD
#define DI_2_Pin GPIO_PIN_6
#define DI_2_GPIO_Port GPIOC
#define DI_1_Pin GPIO_PIN_7
#define DI_1_GPIO_Port GPIOC
#define DI_0_Pin GPIO_PIN_8
#define DI_0_GPIO_Port GPIOC
#define Q_0_Pin GPIO_PIN_15
#define Q_0_GPIO_Port GPIOA
#define Q_1_Pin GPIO_PIN_10
#define Q_1_GPIO_Port GPIOC
#define Q_2_Pin GPIO_PIN_11
#define Q_2_GPIO_Port GPIOC
#define Q_3_Pin GPIO_PIN_12
#define Q_3_GPIO_Port GPIOC
#define Q_4_Pin GPIO_PIN_0
#define Q_4_GPIO_Port GPIOD
#define Q_5_Pin GPIO_PIN_1
#define Q_5_GPIO_Port GPIOD
#define Q_6_Pin GPIO_PIN_2
#define Q_6_GPIO_Port GPIOD
#define WDI_Pin GPIO_PIN_3
#define WDI_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern DAC_HandleTypeDef hdac;

extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
