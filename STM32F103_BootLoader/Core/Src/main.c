/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "menu.h"
#include "flash_if.h"
#include "ymodem.h"
#include "common.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SAVE_FLAG_ADDR 0x0807f864
// #define UPDATE_VALUE 0x5AA51234
/* Start user code address*/
// typedef void (*pFunction)(void);
// pFunction JumpToApplication;
// uint32_t JumpAddress;
uint32_t gJump_ApplicationAddr = 0;
uint32_t gUpdate_flag = 0;
extern pFunction JumpToApplication;
extern uint32_t JumpAddress;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Initialise Flash */
  FLASH_If_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    gUpdate_flag = (*(__IO uint32_t *)UPDATE_SAVE_ADDRESS);
    uint32_t jump_addr = (((gUpdate_flag & 0xFFFF0000) >> 16U) == UPDATE_APP1) ? APPLICATION1_ADDRESS
                                                                               : ((((gUpdate_flag & 0xFFFF0000) >> 16U) == UPDATE_APP2) ? APPLICATION2_ADDRESS : APPLICATION1_ADDRESS);
    /*Record the execution address of the current application*/
    gJump_ApplicationAddr = jump_addr;
    if (((gUpdate_flag & 0x0000FFFF) == UPDATE_CMD) || (!jump_addr))
    {
      Serial_PutString((uint8_t *)"\r\nIAP Start .......\r\n");
      /* Initialise Flash */
      // FLASH_If_Init();
      /* Display main menu */
      Main_Menu();
    }
    else
    {
      /* Test if user code is programmed starting from address "jump_addr" */
      if ((((*(__IO uint32_t *)jump_addr) & 0x2FFE0000) == 0x20000000) && jump_addr)
      {
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t *)(jump_addr + 4);
        if ((JumpAddress & jump_addr) == jump_addr)
        {
          if (jump_addr == APPLICATION1_ADDRESS)
          {
            Serial_PutString((uint8_t *)"\r\nApp1 starts execution ......\r\n");
          }
          else if (jump_addr == APPLICATION2_ADDRESS)
          {
            Serial_PutString((uint8_t *)"\r\nApp2 starts execution ......\r\n");
          }
          JumpToApplication = (pFunction)JumpAddress;
          /* Initialize user application's Stack Pointer */
          __set_MSP(*(__IO uint32_t *)jump_addr);
          /*Close interrupt*/
          __set_FAULTMASK(1);
          JumpToApplication();
        }
        else
        {
          Serial_PutString((uint8_t *)"\r\nError: The application address does not match the jump address!\r\n");
        }
      }
      else
      {
        Serial_PutString((uint8_t *)"\r\nJump address error: please check the interrupt vector and program address !\r\n");
        HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
        // HAL_Delay(500);
        // /*Jump failed. Restart bootloader*/
        // gUpdate_flag = ((uint32_t)(gUpdate_flag & 0xFFFF0000) | UPDATE_CMD);
        // /* erase user application area */
        // FLASH_If_Erase(UPDATE_SAVE_ADDRESS, FLASH_PAGE_SIZE);
        // /*Upgrade success flag will be written in flash*/
        // FLASH_If_Write(UPDATE_SAVE_ADDRESS, &gUpdate_flag, 1U);
        // NVIC_SystemReset();
      }
    }
    Serial_PutString((uint8_t *)"\r\nNote: The system starts to restart......\r\n");
    /*重启系统*/
    HAL_Delay(500);
    /*Jump failed. Restart bootloader*/
    gUpdate_flag = ((uint32_t)(gUpdate_flag & 0xFFFF0000) | UPDATE_CMD);
    /* erase user application area */
    FLASH_If_Erase(UPDATE_SAVE_ADDRESS, FLASH_PAGE_SIZE);
    /*Upgrade success flag will be written in flash*/
    FLASH_If_Write(UPDATE_SAVE_ADDRESS, &gUpdate_flag, 1U);
    NVIC_SystemReset();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
