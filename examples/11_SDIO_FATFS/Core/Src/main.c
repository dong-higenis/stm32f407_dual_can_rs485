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
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FATFS   fs;   // FATFS structure
FIL     file; // File object structure
FRESULT res;  // File function return code
UINT    bytes_written; // The number of bytes actually written
                       // when writing a file in FatFS
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void SD_Init(void);
void SD_WriteFile(void);
void SD_ReadFile(void);
void SD_DeleteFile(void);
void list_files(char *path);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 50);
  return len;
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
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
  printf("===============================\n");
  printf("SD Card Information:\n");
  printf("Block size  : %lu\n", hsd.SdCard.BlockSize);
  printf("Block number: %lu Byte\n", hsd.SdCard.BlockNbr);
  printf("Card size   : %lu GB\n",
         (hsd.SdCard.BlockSize * (hsd.SdCard.BlockNbr / 1024)) / (1024 * 1024));
  printf("Card version: %lu\n", hsd.SdCard.CardVersion);
  printf("===============================\n\n");

  SD_Init();
  list_files("/");
  SD_WriteFile();
  list_files("/");
  SD_ReadFile();
  SD_DeleteFile();
  list_files("/");

  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
void SD_Init(void) {
  // Mount SD Card
  res = f_mount(&fs, "", 1);
  if (res == FR_OK) {
    printf("SD Card Mounted Successfully!\r\n");
  } else {
    printf("SD Card Mount Failed! Error: %d\r\n", res);
  }
}

void SD_WriteFile(void) {
  res = f_open(&file, "test.txt", FA_WRITE | FA_CREATE_ALWAYS);
  if (res == FR_OK) {
    char data[] = "Hello, STM32 SD Card!";
    f_write(&file, data, sizeof(data), &bytes_written);
    f_close(&file);
    printf("File written successfully!\r\n");
  } else {
    printf("File write failed! Error: %d\r\n", res);
  }
}

void SD_ReadFile(void) {
  res = f_open(&file, "test.txt", FA_READ);
  if (res == FR_OK) {
    char buffer[100] = {0};
    f_read(&file, buffer, sizeof(buffer) - 1, &bytes_written);
    f_close(&file);
    printf("Read from file: %s\r\n", buffer);
  } else {
    printf("File read failed! Error: %d\r\n", res);
  }
}

void SD_DeleteFile(void) {
  res = f_unlink("test.txt");
  if (res == FR_OK) {
    printf("File deleted successfully!\r\n");
  } else {
    printf("File delete failed! Error: %d\r\n", res);
  }
}


void list_files(char *path) {
  FRESULT res;
  FILINFO fno;
  DIR dir;

  // Open Directory
  res = f_opendir(&dir, path);
  if (res != FR_OK) {
    printf("Error opening directory: %s, Error Code: %d\r\n", path, res);
    return;
  }

  printf("  Listing files in: %s\r\n", path);

  while (1) {
    res = f_readdir(&dir, &fno);
    if (res != FR_OK || fno.fname[0] == 0) {
      break;  // Error occurred or file not found
    }

    // Check if it is a folder or a file
    if (fno.fattrib & AM_DIR) {
      printf("  - [DIR] %s\r\n", fno.fname);
    } else {
      printf("  - [FILE] %s - %lu bytes\r\n", fno.fname, fno.fsize);
    }
  }
  printf("\n");
  f_closedir(&dir);
}

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
