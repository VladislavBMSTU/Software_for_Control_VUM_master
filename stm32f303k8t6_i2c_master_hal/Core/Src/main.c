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
#include "i2c.h"
#include "gpio.h"

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
// Адреса плат


#define ADDR_PLATA_1 0x20<<1 // Адрес 1 платы 0100 000(7 бит)
#define ADDR_PLATA_2 0x21<<1 // Адрес 2 платы 0100 001(7 бит)
#define ADDR_PLATA_3 0x22<<1 // Адрес 3 платы 0100 010(7 бит)
#define ADDR_PLATA_4 0x23<<1 // Адрес 4 платы 0100 011(7 бит)
#define ADDR_PLATA_5 0x24<<1 // Адрес 5 платы 0100 100(7 бит)
#define ADDR_PLATA_6 0x25<<1 // Адрес 6 платы 0100 101(7 бит)
#define ADDR_PLATA_7 0x26<<1 // Адрес 7 платы 0100 110(7 бит)
#define ADDR_PLATA_8 0x27<<1 // Адрес 8 платы 0100 111(7 бит)

// Регистры для выполнения функций
#define Turn_off      0x01 // Выключить плату
#define Turn_on       0x02 // Включить плату
#define Broken        0x03 // Плата сломана
#define Reset         0x04 // Перезапустить плату
#define ADC_otr       0x05 // АЦП отраж
#define ADC_pad       0x06 // АЦП пад
#define ADC_30v       0x07 // АЦП 30 В
#define ADC_8v        0x08 // АЦП 8 В
#define On_DC_DC      0x09 // Включить DC/DC
#define calib_ADC_otr 0x0A // Калибровка АЦП отраж
#define calib_ADC_pad 0x0B // Калибровка АЦП пад
#define calib_ADC_30v 0x0C // Калибровка АЦП 30 В
#define calib_ADC_8v  0x0D // Калибровка АЦП 8 В
#define Temperature   0x0E // Измерение температуры
#define Power         0x0F // Измерение мощности

// Буффер для команд
uint8_t COMMAND[10] = {calib_ADC_otr, ADC_otr, calib_ADC_pad, ADC_pad, calib_ADC_30v, ADC_30v, calib_ADC_8v, ADC_8v, Temperature, Power};
// Буффер с адресами
uint8_t ADDRESS[8] = {ADDR_PLATA_1, ADDR_PLATA_2, ADDR_PLATA_3, ADDR_PLATA_4, ADDR_PLATA_5, ADDR_PLATA_6, ADDR_PLATA_7, ADDR_PLATA_8};


// Переменные для хранения принятых значений
uint8_t ADC_OTR;
uint8_t ADC_PAD;
uint8_t ADC_30V;
uint8_t ADC_8V;

// Флаг состояния передачи данных
volatile uint8_t transferInProgress = 0;
// Флаг для успешного приёма
volatile uint8_t receiveInProgress = 0;

// Буффер для хранения данных
uint8_t receivedData[8][10];  // [8] - кол-во устройств [10] - кол-во команд

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  // Цикл передачи и приёма данных
	  for (uint8_t i = 0; i < 8; i++)  // номер платы
	  	{
	  		for (uint8_t j = 0; j < 10; j++) // номер команды
	  		{
	  			// Передача [адрес платы = i], j, количество байт.
	  			transferInProgress = 1;
	  			I2C_TRANSMIT_IT(ADDRESS[i], &COMMAND[j], 1);

	  			while (transferInProgress)
	  			{
	  				// Можно добавить здесь тайм-аут для выхода из цикла ожидания в случае ошибки
	  			}

	  			// Приём от слэйва и сохранение значения в переменную.
	  			receiveInProgress = 1;
	  			I2C_RECEIVE_IT(ADDRESS[i], &receivedData[i][j], 1);

	  			while (receiveInProgress)
	  			{
	  				// Можно добавить здесь тайм-аут для выхода из цикла ожидания в случае ошибки
	  			}

	  		}
	  	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Функция передачи данных
void I2C_TRANSMIT_IT(uint8_t DevAddress, uint8_t *pData, uint8_t Size)
{


  if (HAL_I2C_Master_Transmit_IT(&hi2c1, DevAddress, pData, Size) != HAL_OK)
  {
	// Обработка ошибки
	transferInProgress = 0;
    Error_Handler();
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback функция при успешной передаче от мастера слейву
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Проверим, пришла ли команда и проверим ее содержание
    if (hi2c->Instance->ISR & I2C_ISR_ADDR)
    {

    	  transferInProgress = 0;


    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Функция приёма данных
void I2C_RECEIVE_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    if (HAL_I2C_Master_Receive_IT(&hi2c1, DevAddress, pData, Size) != HAL_OK)
    {
        // Обработка ошибки
        receiveInProgress = 0;
        Error_Handler();
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback функция при успешном приеме данных от слейва к мастеру
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	// Добавить функцию проверки успешного приёма данных
    receiveInProgress = 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
