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
#include "tim.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 50);
  return len;
}

void delay_us(uint32_t us) {
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while((__HAL_TIM_GET_COUNTER(&htim2) - start) < us);
}


// Setting GPIO output mode
void DS18B20_GPIO_Output(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}

// Setting GPIO input mode (Pull-up)
void DS18B20_GPIO_Input(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);
}

// **1-Wire Reset (Reset pulse & Presence detect)**
uint8_t DS18B20_Reset(void) {
    uint8_t presence = 0;

    DS18B20_GPIO_Output();
    HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
    delay_us(480);

    DS18B20_GPIO_Input();
    delay_us(70);

    presence = HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin);
    delay_us(410);

    return (presence == GPIO_PIN_RESET) ? 1 : 0;
}

// **1-Wire Write data (Write '1' or '0')**
void DS18B20_WriteBit(uint8_t bit) {
    DS18B20_GPIO_Output();
    HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);

    if (bit) {
        delay_us(1);
        DS18B20_GPIO_Input();
        delay_us(59);
    } else {
        delay_us(60);
        DS18B20_GPIO_Input();
    }
}

// **1-Wire Read data**
uint8_t DS18B20_ReadBit(void) {
    uint8_t bit = 0;

    DS18B20_GPIO_Output();
    HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
    delay_us(1);

    DS18B20_GPIO_Input();
    delay_us(14);

    bit = HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin);
    delay_us(45);

    return bit;
}

// **1-Wire Write bytes**
void DS18B20_WriteByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        DS18B20_WriteBit(byte & 0x01);
        byte >>= 1;
    }
}

// **1-Wire Read bytes**
uint8_t DS18B20_ReadByte(void) {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (DS18B20_ReadBit()) {
            byte |= (1 << i);
        }
    }
    return byte;
}

// **Temperature reading function**
float DS18B20_GetTemperature(void) {
    uint8_t temp_lsb, temp_msb;
    int16_t temp_raw;

    if (!DS18B20_Reset()) return -1000;

    DS18B20_WriteByte(0xCC);
    DS18B20_WriteByte(0x44);

    HAL_Delay(750);

    DS18B20_Reset();
    DS18B20_WriteByte(0xCC);
    DS18B20_WriteByte(0xBE);

    temp_lsb = DS18B20_ReadByte();
    temp_msb = DS18B20_ReadByte();

    temp_raw = (temp_msb << 8) | temp_lsb;

    return temp_raw / 16.0;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    float temperature = DS18B20_GetTemperature();
    if (temperature != -1000) {
        printf("Temperature: %.2f°C\n", temperature);
    } else {
        printf("DS18B20 not detected!\n");
    }
    HAL_Delay(1000);

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
