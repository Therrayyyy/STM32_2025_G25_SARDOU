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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "lcd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
// LCD variable
rgb_lcd lcd;

int weightMg;
float weightG;

// LoRa command and response buffers
char loraCmd[64];
char loraResp[256];
char loraMsg[64];
char loraRetMsg[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void displayData(char *message);
void initializePeripherals(void);
void sendLoRaCommand(const char* cmd);
void clearBuffers(void);

/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
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
  initializePeripherals();

  // Send initial LoRa commands
  sendLoRaCommand("AT\r\n");
  sendLoRaCommand("AT+MODE=TEST\r\n");

  // Set LoRa module to receive mode
  sendLoRaCommand("AT+TEST=RXLRPKT\r\n");

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    // Wait and retrieve the message
    HAL_UART_Receive(&huart1, (uint8_t*)loraRetMsg, sizeof(loraRetMsg), 1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)loraRetMsg, strlen(loraRetMsg), 1000);

    // Check if the received message contains "+TEST: RX "
    char *start = strstr(loraRetMsg, "+TEST: RX \"");
    if (start != NULL)
    {
        start += 11; // Move past "+TEST: RX \""
        displayData(start);
    }

    // Clear buffers to avoid stale data
    clearBuffers();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Initialize peripherals and LCD.
  */
void initializePeripherals(void)
{
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();

  lcd_init(&hi2c1, &lcd);
  clearlcd();
}

/**
  * @brief Send a command to the LoRa module.
  */
void sendLoRaCommand(const char* cmd)
{
  snprintf(loraCmd, sizeof(loraCmd), "%s", cmd);
  HAL_UART_Transmit(&huart1, (uint8_t*)loraCmd, strlen(loraCmd), 1000);
  HAL_UART_Receive(&huart1, (uint8_t*)loraResp, sizeof(loraResp), 1000);
}

/**
  * @brief Clear the command and response buffers.
  */
void clearBuffers(void)
{
  memset(loraMsg, 0, sizeof(loraMsg));
  memset(loraRetMsg, 0, sizeof(loraRetMsg));
}

/**
  * @brief Display data on the LCD based on the message ID.
  */
void displayData(char *message) {
    char buff[20];

    // Check if the message is from the external temperature and humidity sensor (ID "01")
    if (strncmp(message, "01", 2) == 0) {
        clearlcd();
        lcd_position(&hi2c1, 0, 0);
        lcd_print(&hi2c1, "Temp & Humidity");
        lcd_position(&hi2c1, 0, 1);
        lcd_print(&hi2c1, "Outside");
        HAL_Delay(2000);
        clearlcd();

        // Extract and display temperature
        snprintf(buff, sizeof(buff), "Temp: %c%c.%c%c C", message[2], message[3], message[4], message[5]);
        lcd_position(&hi2c1, 0, 0);
        lcd_print(&hi2c1, buff);
        memset(buff, 0, sizeof(buff));

        // Extract and display humidity
        snprintf(buff, sizeof(buff), "Hum: %c%c.%c%c %%", message[6], message[7], message[8], message[9]);
        lcd_position(&hi2c1, 0, 1);
        lcd_print(&hi2c1, buff);
        memset(buff, 0, sizeof(buff));
    }

    // Check if the message is from the weight sensor (ID "02")
    else if (strncmp(message, "02", 2) == 0) {
        weightMg = atoi(&message[2]); // Convert string to integer (mg)
        weightG = weightMg / 1000.0; // Convert to grams

        clearlcd();
        lcd_position(&hi2c1, 2, 0);
        lcd_print(&hi2c1, "Current Weight");
        lcd_position(&hi2c1, 2, 1);
        lcd_print(&hi2c1, "in Hive");
        HAL_Delay(2000);
        clearlcd();

        lcd_position(&hi2c1, 0, 0);
        lcd_print(&hi2c1, "Hive Weight:");
        snprintf(buff, sizeof(buff), "%.2f g", weightG);
        lcd_position(&hi2c1, 0, 1);
        lcd_print(&hi2c1, buff);
        memset(buff, 0, sizeof(buff));
    }

    // Check for an alternative format of the weight sensor (ID "002")
    else if (strncmp(message, "002", 3) == 0) {
        weightMg = atoi(&message[3]);
        weightG = weightMg / 1000.0;

        clearlcd();
        lcd_position(&hi2c1, 2, 0);
        lcd_print(&hi2c1, "Current Weight");
        lcd_position(&hi2c1, 2, 1);
        lcd_print(&hi2c1, "in Hive");
        HAL_Delay(2000);
        clearlcd();

        lcd_position(&hi2c1, 0, 0);
        lcd_print(&hi2c1, "Hive Weight:");
        snprintf(buff, sizeof(buff), "%.2f g", weightG);
        lcd_position(&hi2c1, 0, 1);
        lcd_print(&hi2c1, buff);
        memset(buff, 0, sizeof(buff));
    }

    // Check if the message is from the internal temperature and humidity sensor (ID "03")
    else if (strncmp(message, "03", 2) == 0) {
        clearlcd();
        lcd_position(&hi2c1, 0, 0);
        lcd_print(&hi2c1, "Temp & Humidity");
        lcd_position(&hi2c1, 0, 1);
        lcd_print(&hi2c1, "Inside");
        HAL_Delay(2000);
        clearlcd();

        // Extract and display temperature
        snprintf(buff, sizeof(buff), "Temp: %c%c.%c C", message[2], message[3], message[4]);
        lcd_position(&hi2c1, 0, 0);
        lcd_print(&hi2c1, buff);
        memset(buff, 0, sizeof(buff));

        // Extract and display humidity
        snprintf(buff, sizeof(buff), "Hum: %c%c.%c %%", message[5], message[6], message[7]);
        lcd_position(&hi2c1, 0, 1);
        lcd_print(&hi2c1, buff);
        memset(buff, 0, sizeof(buff));
    }

    // Check if motion is detected (ID "04")
    else if (strncmp(message, "04", 2) == 0) {
        clearlcd();
        lcd_position(&hi2c1, 2, 0);
        lcd_print(&hi2c1, "PRESENCE");
        lcd_position(&hi2c1, 1, 1);
        lcd_print(&hi2c1, "DETECTED");
    }
}

PUTCHAR_PROTOTYPE // Display printf in terminal
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
