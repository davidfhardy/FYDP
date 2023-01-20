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
void osc_write_to_register(uint8_t REG, uint8_t VAL);
void osc_read_register(uint8_t REG, char NAME[20]);
void osc_init(void);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char spi_buf[20];
int uart_buf_len;
char uart_buf[50];

/* Oscillator Registers ------------------------------------------------------*/

// READ
const uint8_t R_REG0 = 0b00000001;
const uint8_t R_REG1 = 0b00000011;
const uint8_t R_REG2 = 0b00000101;
const uint8_t R_REG3 = 0b00000111;
const uint8_t R_REG4 = 0b00001001;
const uint8_t R_REG5 = 0b00001011;
const uint8_t R_REG6 = 0b00001101;
const uint8_t R_REG7 = 0b00001111;
const uint8_t R_REG8 = 0b00010001;
const uint8_t R_REG9 = 0b00010011;
const uint8_t R_REGA = 0b00010101;
const uint8_t R_REGB = 0b00010111;
const uint8_t R_REGC = 0b00011001;
const uint8_t R_REGD = 0b00011011;
const uint8_t R_REGE = 0b00011111;

// WRITE
const uint8_t W_REG0 = 0b00000000;
const uint8_t W_REG1 = 0b00000010;
const uint8_t W_REG2 = 0b00000100;
const uint8_t W_REG3 = 0b00000110;
const uint8_t W_REG4 = 0b00001000;
const uint8_t W_REG5 = 0b00001010;
const uint8_t W_REG6 = 0b00001100;
const uint8_t W_REG7 = 0b00001110;
const uint8_t W_REG8 = 0b00010000;
const uint8_t W_REG9 = 0b00010010;
const uint8_t W_REGA = 0b00010100;
const uint8_t W_REGB = 0b00010110;
const uint8_t W_REGC = 0b00011000;
const uint8_t W_REGD = 0b00011010;
const uint8_t W_REGE = 0b00011110;

// Setting registers
//const uint8_t VAL_REG2 = 0b00000100; // 4
//const uint8_t VAL_REG4 = 0b01011110; //94
//const uint8_t VAL_REG6 = 0x14;
//const uint8_t VAL_REG7 = 81;
//const uint8_t VAL_REG8 = 0x0A;
//const uint8_t VAL_REG9 = 0x3D;
//const uint8_t VAL_REGA = 0x70;
//const uint8_t VAL_REGB = 26;
//const uint8_t VAL_REGD = 0xE0;

const uint8_t VAL_REG1 = 0x04;
const uint8_t VAL_REG2 = 0x04;
const uint8_t VAL_REG3 = 0x3F;
const uint8_t VAL_REG4 = 0x57;
const uint8_t VAL_REG5 = 0x11;
const uint8_t VAL_REG6 = 0x18;
const uint8_t VAL_REG7 = 40;
const uint8_t VAL_REG8 = 0x3F;
const uint8_t VAL_REG9 = 0xFF;
const uint8_t VAL_REGA = 0xF0;
const uint8_t VAL_REGB = 0x19;
const uint8_t VAL_REGC = 0xBF;
const uint8_t VAL_REGD = 0x00; //C0

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
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
//  char uart_buf[50];
//  int uart_buf_len;
  // char spi_buf[20];
  for(int i=0; i<20; i++)
  {
	  spi_buf[i] = 'a';
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  uart_buf_len = sprintf(uart_buf, "========== Begin Running Code ==========\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

  /* BEGIN OSCILLATOR SECTION -----------------------------------------------------*/
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // Should be high by default

  uart_buf_len = sprintf(uart_buf, "/* Oscillator Code ---------*/ \r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

  // Set oscillator initial values
  // osc_init();
  osc_write_to_register(W_REG1, VAL_REG1);
  osc_write_to_register(W_REG2, VAL_REG2);
  osc_write_to_register(W_REG3, VAL_REG3);
  osc_write_to_register(W_REG4, VAL_REG4);
  osc_write_to_register(W_REG5, VAL_REG5);
  osc_write_to_register(W_REG6, VAL_REG6);
  osc_write_to_register(W_REG7, VAL_REG7);
  osc_write_to_register(W_REG8, VAL_REG8);
  osc_write_to_register(W_REG9, VAL_REG9);
  osc_write_to_register(W_REGA, VAL_REGA);
  osc_write_to_register(W_REGB, VAL_REGB);
  osc_write_to_register(W_REGC, VAL_REGC);
  osc_write_to_register(W_REGD, VAL_REGD);

  // Read register values
  osc_read_register(R_REG0, "REG0");
  osc_read_register(R_REG1, "REG1");
  osc_read_register(R_REG2, "REG2");
  osc_read_register(R_REG3, "REG3");
  osc_read_register(R_REG4, "REG4");
  osc_read_register(R_REG5, "REG5");
  osc_read_register(R_REG6, "REG6");
  osc_read_register(R_REG7, "REG7");
  osc_read_register(R_REG8, "REG8");
  osc_read_register(R_REG9, "REG9");
  osc_read_register(R_REGA, "REGA");
  osc_read_register(R_REGB, "REGB");
  osc_read_register(R_REGC, "REGC");
  osc_read_register(R_REGD, "REGD");

  uart_buf_len = sprintf(uart_buf, "=========== End Code Running ===========\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

  /* END OSCILLATOR SECTION -----------------------------------------------------*/

  /* Scratch Work ---------------------------------------------------------------*/
  /*
   // read initial value
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&R_REG2, 1, 100);
  HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  uart_buf_len = sprintf(uart_buf, "REG2 Value: 0x%02x\r\n", (unsigned int)spi_buf[0]);
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
  uart_buf_len = sprintf(uart_buf, "SPI Test end\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

  //write
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&W_REG2, 1, 100);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&VAL_REG2, 1, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  //read changed value
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&R_REG2, 1, 100);
  HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  uart_buf_len = sprintf(uart_buf, "REG2 Value: 0x%02x\r\n", (unsigned int)spi_buf[0]);
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

   */
  /* End Scratch Work -----------------------------------------------------------*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void osc_write_to_register(uint8_t REG, uint8_t VAL)
{
	uint8_t tx_data[2];

	tx_data[0] = REG;
	tx_data[1] = VAL;


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)tx_data, 2, 100);
	//HAL_SPI_Transmit(&hspi1, (uint8_t *)&VAL, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void osc_read_register(uint8_t REG, char NAME[20])
{
	uint8_t tx_data[2];
	uint8_t rx_data[2];

	tx_data[0] = REG;
	tx_data[1] = 0;
	rx_data[0] = 0;
	rx_data[1] = 0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	// HAL_SPI_Transmit(&hspi1, (uint8_t *)&REG, 1, 100);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	uart_buf_len = sprintf(uart_buf, "%s Value: 0x%02x\r\n", NAME, (unsigned int)rx_data[1]);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}

void osc_init(void)
{
	/* Initiates register values in the oscillator */
//	osc_write_to_register(W_REG1, VAL_REG1);
//	osc_write_to_register(W_REG2, VAL_REG2);
//	osc_write_to_register(W_REG4, VAL_REG4);
//	osc_write_to_register(W_REG5, VAL_REG5);
//	osc_write_to_register(W_REG6, VAL_REG6);
//	osc_write_to_register(W_REG7, VAL_REG7);
//	osc_write_to_register(W_REG8, VAL_REG8);
	osc_write_to_register(W_REG9, VAL_REG9);
//	osc_write_to_register(W_REGA, VAL_REGA);
//	osc_write_to_register(W_REGB, VAL_REGB);
//	osc_write_to_register(W_REGC, VAL_REGC);
//	osc_write_to_register(W_REGD, VAL_REGD);
//	osc_write_to_register(W_REG3, VAL_REG3); // autocal at the end of rest of registers

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
