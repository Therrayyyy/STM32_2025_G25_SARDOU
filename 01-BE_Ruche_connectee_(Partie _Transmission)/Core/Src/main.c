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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "dht22.h"
#include "hx711.h"
#include <stdio.h>
#include <string.h>
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
// LoRa variables
char loraCmd[64];
char loraResp[256];
char loraMsg[256];
char loraRetMsg[256];

// SHT31 variables
uint8_t shtCmd[2] = {0x21, 0x26};
uint8_t shtData[6];
uint16_t rawTemp;
uint16_t rawHum;
float temp1, hum1;

// HX711 variables - Calibration values
uint32_t tareVal = 0;
float knownWeight = 500000;  // in milligrams
float knownHxVal = 1;
int weightVal;

// DHT22 variables
float temp2, hum2;

// LCD variable
rgb_lcd lcd;

// Motion detection variable
volatile int motionDetected = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void initializeSensors(void);
void sendLoRaCommand(const char* cmd);
void displayLCDMessage(const char* line1, const char* line2);
void processSHT31Data(void);
void processDHT22Data(void);
void processHX711Data(void);

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
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  initializeSensors();

  // Send initial LoRa commands
  sendLoRaCommand("AT\r\n");
  sendLoRaCommand("AT+MODE=TEST\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    processSHT31Data();
    HAL_Delay(20000);

    processHX711Data();
    HAL_Delay(10000);

    processDHT22Data();
    HAL_Delay(20000);

  }
  /* USER CODE END 3 */
}

/**
  * @brief Initialize sensors and peripherals.
  */
void initializeSensors(void)
{
  HAL_TIM_Base_Start(&htim6); // Start the timer
  HX711_Start();

  // Initialize LCD
  lcd_init(&hi2c1, &lcd);
  displayLCDMessage("Calibration", "Weight Sensor");
  HAL_Delay(3000);

  // Start auto-calibration
  calibrate(&knownHxVal, &tareVal);
  displayLCDMessage("Calibration", "Complete");
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
  * @brief Display a message on the LCD.
  */
void displayLCDMessage(const char* line1, const char* line2)
{
  clearlcd();
  lcd_position(&hi2c1, 0, 0);
  lcd_print(&hi2c1, line1);
  lcd_position(&hi2c1, 0, 1);
  lcd_print(&hi2c1, line2);
}

/**
  * @brief Process data from the SHT31 sensor.
  */
void processSHT31Data(void)
{
  // Send I2C command to configure SHT31 sensor
  HAL_I2C_Master_Transmit(&hi2c1, (0x44 << 1), shtCmd, 2, 5000);
  HAL_Delay(50);

  // Read data from SHT31 sensor
  HAL_I2C_Master_Receive(&hi2c1, (0x44 << 1), shtData, 6, 5000);
  HAL_Delay(50);

  // Convert raw data to temperature and humidity
  rawTemp = (shtData[0] << 8) | shtData[1];
  temp1 = -45 + 175 * (rawTemp / 65535.0);
  rawHum = (shtData[3] << 8) | shtData[4];
  hum1 = 100 * (rawHum / 65535.0);

  // Display data on LCD
  char dispTemp1[20], dispHum1[20];
  sprintf(dispTemp1, "Temp: %.2f C", temp1);
  sprintf(dispHum1, "Hum: %.2f %%", hum1);
  displayLCDMessage(dispTemp1, dispHum1);

  // Prepare and send LoRa message
  char sensor1Id[3] = {"01"};
  int tempInt1 = (int)temp1;
  float tempDec1 = temp1 - tempInt1;
  int tempDecInt1 = (int)(tempDec1 * 100);
  int humInt1 = (int)hum1;
  float humDec1 = hum1 - humInt1;
  int humDecInt1 = (int)(humDec1 * 100);

  snprintf(loraMsg, sizeof(loraMsg), "AT+TEST=TXLRPKT \"%s%02d%02d%02d%02d\"\r\n", sensor1Id, tempInt1, tempDecInt1, humInt1, humDecInt1);
  HAL_UART_Transmit(&huart1, (uint8_t*)loraMsg, strlen(loraMsg), 1000);
  HAL_UART_Receive(&huart1, (uint8_t*)loraRetMsg, sizeof(loraRetMsg), 1000);

  // Clear buffers
  memset(loraMsg, 0, sizeof(loraMsg));
  memset(loraRetMsg, 0, sizeof(loraRetMsg));
}

/**
  * @brief Process data from the DHT22 sensor.
  */
void processDHT22Data(void)
{
  DHT22_Get_Data(&temp2, &hum2);

  // Display data on LCD
  char dispTemp2[20], dispHum2[20];
  sprintf(dispTemp2, "Temp: %.1f C", temp2);
  sprintf(dispHum2, "Hum: %.1f %%", hum2);
  displayLCDMessage(dispTemp2, dispHum2);

  // Prepare and send LoRa message
  char sensor3Id[3] = {"03"};
  int tempInt2 = (int)temp2;
  float tempDec2 = temp2 - tempInt2;
  int tempDecInt2 = (int)(tempDec2 * 10);
  int humInt2 = (int)hum2;
  float humDec2 = hum2 - humInt2;
  int humDecInt2 = (int)(humDec2 * 10);

  snprintf(loraMsg, sizeof(loraMsg), "AT+TEST=TXLRPKT \"%s%02d%01d%02d%01d\"\r\n", sensor3Id, tempInt2, tempDecInt2, humInt2, humDecInt2);
  HAL_UART_Transmit(&huart1, (uint8_t*)loraMsg, strlen(loraMsg), 1000);
  HAL_UART_Receive(&huart1, (uint8_t*)loraRetMsg, sizeof(loraRetMsg), 1000);

  // Clear buffers
  memset(loraMsg, 0, sizeof(loraMsg));
  memset(loraRetMsg, 0, sizeof(loraRetMsg));
}

/**
  * @brief Process data from the HX711 sensor.
  */
void processHX711Data(void)
{
  char sensor2Id[3] = {"02"};
  char loraWeight[20];

  weightVal = weigh(knownWeight, knownHxVal, tareVal);

  if (weightVal > 0) {
      sprintf(loraWeight, "%d", weightVal);

      // Construct and send LoRa message
      snprintf(loraMsg, sizeof(loraMsg), "AT+TEST=TXLRPKT \"%s%s\"\r\n", sensor2Id, loraWeight);
      HAL_UART_Transmit(&huart1, (uint8_t*)loraMsg, strlen(loraMsg), 1000);
      HAL_UART_Receive(&huart1, (uint8_t*)loraRetMsg, sizeof(loraRetMsg), 1000);

      // Clear buffers
      memset(loraMsg, 0, sizeof(loraMsg));
      memset(loraRetMsg, 0, sizeof(loraRetMsg));
  }
}

PUTCHAR_PROTOTYPE // Display printf in terminal
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_8) // If interrupt from PA8 (PIR sensor)
    {
        motionDetected = 1; // Signal motion detected
    }
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
