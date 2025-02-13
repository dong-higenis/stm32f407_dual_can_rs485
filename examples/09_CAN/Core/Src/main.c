/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN1_ID 0x111   // CAN1 transfer ID
#define CAN2_ID 0x222   // CAN2 transfer ID
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

// Test data
uint8_t can1_tx_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
uint8_t can2_tx_data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x99, 0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Config(void);
void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t length);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
int _write(int file, char *ptr, int len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}


/* CAN Configuration */
void CAN_Config()
{
    CAN_FilterTypeDef canFilter1;

    // Filter settings (common settings for CAN1 & CAN2)
    canFilter1.FilterBank = 0;
    canFilter1.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter1.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilter1.FilterIdHigh = 0x0000;
    canFilter1.FilterIdLow = 0x0000;
    canFilter1.FilterMaskIdHigh = 0x0000;
    canFilter1.FilterMaskIdLow = 0x0000;
    canFilter1.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilter1.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &canFilter1) != HAL_OK)
    {
      printf("CAN1 filter setup failed!\n");
    }

    CAN_FilterTypeDef canFilter2;

    // Filter settings (common settings for CAN1 & CAN2)
    canFilter2.FilterBank = 1;
    canFilter2.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter2.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilter2.FilterIdHigh = 0x0000;
    canFilter2.FilterIdLow = 0x0000;
    canFilter2.FilterMaskIdHigh = 0x0000;
    canFilter2.FilterMaskIdLow = 0x0000;
    canFilter2.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilter2.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan2, &canFilter2) != HAL_OK)
    {
      printf("CAN2 filter setup failed!\n");
    }

    // Start CAN1 & CAN2
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);

    // Enable receive interrupt
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        printf("CAN1 Interrupt enable failed!\n");
    }

    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        printf("CAN2 Interrupt enable failed!\n");
    }
}

/* Sending CAN messages */
void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t length)
{
    TxHeader.StdId = id;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = length;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox) != HAL_OK)
    {
        printf("CAN Send Error!\n");
    }
}

/* CAN receive interrupt callback function */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        if (hcan == &hcan1)
        {
            printf("CAN1 Receive: ID=0x%lX, Data: ", RxHeader.StdId);
            for (int i = 0; i < RxHeader.DLC; i++)
            {
                printf("%02X ", RxData[i]);
            }
            printf("\n[O] CAN1: Receive normal data!\n\n");
        }
        else if (hcan == &hcan2)
        {
            printf("CAN2 Receive: ID=0x%lX, Data: ", RxHeader.StdId);
            for (int i = 0; i < RxHeader.DLC; i++)
            {
                printf("%02X ", RxData[i]);
            }
            printf("\n[O] CAN2: Receive normal data!\n\n");
        }
    }
    else
    {
        printf("[X] CAN RX Failed to read message!\n");
    }
}

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  printf("STM32 CAN1 <-> CAN2 Test\n");

  CAN_Config();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    printf("Send CAN1 -> CAN2\n");
    CAN_Send(&hcan1, CAN1_ID, can1_tx_data, 8);
    HAL_Delay(2000);

    printf("Send CAN2 -> CAN1\n");
    CAN_Send(&hcan2, CAN2_ID, can2_tx_data, 8);
    HAL_Delay(2000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
