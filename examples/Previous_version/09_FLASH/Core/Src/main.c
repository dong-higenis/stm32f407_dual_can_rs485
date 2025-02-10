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
#include "spi.h"
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
#define FLASH_CS_LOW()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define FLASH_CS_HIGH() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  if(HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10) != HAL_OK)
  {
    return -1;
  }
  return ch;
}

uint8_t rxbuf;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // to UART1
    HAL_UART_Transmit(huart, (uint8_t *)&rxbuf, 1, 10);
    HAL_UART_Receive_IT(huart, &rxbuf, 1);
  }
}


// 데이터 전송
uint8_t SPI_TransmitReceive(uint8_t data) {
    uint8_t receivedData = 0;
    HAL_SPI_TransmitReceive(&hspi2, &data, &receivedData, 1, HAL_MAX_DELAY);
    return receivedData;
}

// W25Q128 JEDEC ID 읽기
void W25Q128_ReadID(void) {
    uint8_t cmd = 0x9F;
    uint8_t id[3] = {0};

    FLASH_CS_LOW();
    SPI_TransmitReceive(cmd);
    id[0] = SPI_TransmitReceive(0x00);
    id[1] = SPI_TransmitReceive(0x00);
    id[2] = SPI_TransmitReceive(0x00);
    FLASH_CS_HIGH();

    printf("\nManufacturer ID: 0x%X, Memory Type: 0x%X, Capacity: 0x%X\n", id[0], id[1], id[2]);
}

// W25Q128 페이지 프로그램
void W25Q128_WriteData(uint32_t address, uint8_t *data, uint16_t size) {
    uint8_t cmd[4];

    // 쓰기 활성화 명령
    FLASH_CS_LOW();
    SPI_TransmitReceive(0x06);
    FLASH_CS_HIGH();

    // 페이지 프로그램 명령
    cmd[0] = 0x02;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, data, size, HAL_MAX_DELAY);
    FLASH_CS_HIGH();
}

// W25Q128 데이터 읽기
void W25Q128_ReadData(uint32_t address, uint8_t *buffer, uint16_t size) {
    uint8_t cmd[4];

    // 빠른 읽기 명령
    cmd[0] = 0x0B;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x00}, 1, HAL_MAX_DELAY); // 더미 바이트
    HAL_SPI_Receive(&hspi2, buffer, size, HAL_MAX_DELAY);
    FLASH_CS_HIGH();
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
  MX_USART1_UART_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  FLASH_CS_HIGH();

  uint8_t writeData[16] = "Hello, W25Q128!";
  uint8_t readData[16] = {0};

  // JEDEC ID 읽기
  W25Q128_ReadID();

  // 데이터 쓰기
  W25Q128_WriteData(0x000000, writeData, sizeof(writeData));
  HAL_Delay(50); // 쓰기 완료 대기

  // 데이터 읽기
  W25Q128_ReadData(0x000000, readData, sizeof(readData));

  // 읽은 데이터 출력
  printf("Read Data: %s\n", readData);
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
