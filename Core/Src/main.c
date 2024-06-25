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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "tilt.h"
#include "adc.h"
#include "lora.h"


#include "string.h"
#include "stdio.h"
#include "lora.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
adc_t adc1;
adc_t adc2;
adc_t adc3;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

<<<<<<<< HEAD:AMR_receiver/receiver/Core/Src/main.c
uint8_t hello[15] = "Hello, World!\n\r";
uint8_t new_line[1] = "\n";


char uart_buf[50];
uint8_t uart_buf_len;
int ESC = 0x1B;
uint8_t rx_buffer[256];

========
>>>>>>>> f46711b7818af0db365b4cb3413287fc9408f5cf:AMR-CPU/Core/Src/main.c

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
float read_battery(ADC_HandleTypeDef *hadc);
void transmit_data(SPI_HandleTypeDef* spi, char* data);
void get_measurements(SPI_HandleTypeDef* spi, adc_t adcx, adc_t adcy, adc_t adcz, char* data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void clrscr();
void gotoxy(uint8_t x, uint8_t y);
void uart_read(void);
void enter_lpr_mode();
void exit_lpr_mode();

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
<<<<<<<< HEAD:AMR_receiver/receiver/Core/Src/main.c
  char send_data[200];
  uint8_t rx, rx1, rx2, rx_flag, reg_value, send_it, cnt = 0;
  uint8_t tx, tx1 = 0;
  uint8_t addr, addr2, addr3 = 0;
  uint8_t mode;

  char data_read[100];

  uint8_t uart_buf_len = 0;

  HAL_GPIO_WritePin(GPIOA, RST_LoRa_Pin, GPIO_PIN_RESET);
  LoRa_reset();


  clrscr();
  gotoxy(0,0);

  LoRa_init(&hspi1);

  HAL_GPIO_WritePin(GPIOA, TX_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, RX_Pin, GPIO_PIN_SET);

  LoRa_write_reg(&hspi1, RegFiFoTxBaseAddr, 0x00);
  HAL_Delay(10);

  //LoRa_fill_fifo(&hspi1, text, strlen(text));

  LoRa_set_mode(&hspi1, rx_cont_mode);
  mode = 1;
========
  HAL_PWREx_EnableLowPowerRunMode(); // enable low power run
  LSM9DS1_Init(&hspi1);
  ADC_Init(&adc1,&adc2,&adc3);
  //LoRa_reset();
  LoRa_init(&hspi1);

  HAL_GPIO_WritePin(GPIOA, TX_Pin, GPIO_PIN_SET);  // Enable TX path
  HAL_GPIO_WritePin(GPIOA, RX_Pin,GPIO_PIN_RESET); // Disable RX path

  char tx_data[100];
  float battery;
  uint8_t low_battery = 0;
  uint8_t first = 1; // To check for first measurement to bypass delay in first while loop

  // Initialize delay
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = 1500; // Time to wait between each measurement + transmission
  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY){
  	  wait += (uint32_t)(uwTickFreq);
  	  }

>>>>>>>> f46711b7818af0db365b4cb3413287fc9408f5cf:AMR-CPU/Core/Src/main.c

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  battery = read_battery(&hadc1);

	  if(battery < 1.0 && low_battery){
		  break;
	  }	else if(battery < 1.0){
		  low_battery = 1;
		  strcpy(tx_data, "");
		  sprintf(tx_data,"Battery low");
		  transmit_data(&hspi1, tx_data);
		  break;
	  } else if(!((HAL_GetTick() - tickstart) < wait) || first){
		  if(first){
			  first = 0;
			  HAL_Delay(1000);
		  }

		  low_battery = 0;
		  // Set reset of AMR-sensors
		  HAL_GPIO_WritePin(GPIOA, AMR_RESET_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, AMR_RESET_Pin,GPIO_PIN_RESET);

		  // Sensor measurements, calculations and transmission of data
		  get_measurements(&hspi1,adc3,adc2,adc1,tx_data);
		  transmit_data(&hspi1, tx_data);

		  // Restart delay
		  tickstart = HAL_GetTick();
		  wait = 1500;
		  if (wait < HAL_MAX_DELAY){
			  wait += (uint32_t)(uwTickFreq);
			  }
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  gotoxy(0,0);

	  /*
	  //LoRa receive and print and send for 2-way com
	  if (mode != 1) {
		  toggle_pins(&hspi1);
		  LoRa_set_mode(&hspi1, rx_cont_mode);
		  mode = 1;
		  rx = 0;
		  clrscr();
	  }

	  rx = (LoRa_read_reg(&hspi1, RegIrqFlags) & 0x40) >> 6;

	  LoRa_read_payload(&hspi1, data_read);

	  rx1 = LoRa_read_reg(&hspi1, RegRxNbBytes);
	  rx2 = LoRa_read_reg(&hspi1, RegOpMode);

	  uart_buf_len = sprintf(send_data, "Waiting, RegOpMode: %d, mode: %d\n\r", rx2, mode);
	  HAL_UART_Transmit(&huart2, (uint8_t*)send_data, uart_buf_len, 500);

	  if (rx) {
		  uart_buf_len = sprintf(send_data, "Payload: %s, rx_done: %d, bytes: %d", data_read,  rx, rx1);
		  HAL_UART_Transmit(&huart2, (uint8_t*)send_data, uart_buf_len, 500);
		  HAL_Delay(5000);
		  clrscr();

		  for (int i = 0; i <= 2; i++) {
			  LoRa_fill_fifo(&hspi1, text, strlen(text));
			  toggle_pins(&hspi1);
			  LoRa_set_mode(&hspi1, tx_mode);
			  mode = 0;
			  reg_value = LoRa_read_reg(&hspi1, RegOpMode);
			  tx = (LoRa_read_reg(&hspi1, RegIrqFlags) & 0x08) >> 3;
			  if (tx) {
				  send_it = 1;
				  LoRa_write_reg(&hspi1, RegIrqFlags, 0x08);
				  tx = 0;
				  sprintf(send_data, "send it: %d, mode: %d", send_it, reg_value);
				  HAL_UART_Transmit(&huart2, (uint8_t*)send_data, strlen(send_data), 500);
				  send_it = 0;
			  }

			  HAL_Delay(1000);
		  }
		  LoRa_set_mode(&hspi1, sleep_mode);
		  mode = 0;
		  HAL_Delay(10);
	  }*/


	  //LoRa receive and print
	  rx = (LoRa_read_reg(&hspi1, RegIrqFlags) & 0x40) >> 6;

	  rx1 = LoRa_read_reg(&hspi1, RegRxNbBytes);
	  rx2 = LoRa_read_reg(&hspi1, RegOpMode);

	  uart_buf_len = sprintf(send_data, "Waiting, RegOpMode: %d\n\r", rx2);
	  HAL_UART_Transmit(&huart2, (uint8_t*)send_data, uart_buf_len, 500);

	  if (rx) {
		  LoRa_read_payload(&hspi1, data_read);
		  clrscr();
		  uart_buf_len = sprintf(send_data, "Payload:\n\r%s\n\rbytes: %d", data_read, rx1);
		  HAL_UART_Transmit(&huart2, (uint8_t*)send_data, uart_buf_len, 500);
		  LoRa_set_mode(&hspi1, sleep_mode);
		  HAL_Delay(10);
		  LoRa_set_mode(&hspi1, rx_cont_mode);
	  }



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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV64;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
<<<<<<<< HEAD:AMR_receiver/receiver/Core/Src/main.c
========
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_1;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 3972;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
>>>>>>>> f46711b7818af0db365b4cb3413287fc9408f5cf:AMR-CPU/Core/Src/main.c
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
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
<<<<<<<< HEAD:AMR_receiver/receiver/Core/Src/main.c
  HAL_GPIO_WritePin(CS_LoRa_GPIO_Port, CS_LoRa_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TX_Pin|RST_LoRa_Pin|RX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_LoRa_Pin TX_Pin RST_LoRa_Pin RX_Pin */
  GPIO_InitStruct.Pin = CS_LoRa_Pin|TX_Pin|RST_LoRa_Pin|RX_Pin;
========
  HAL_GPIO_WritePin(GPIOA, CSAG_Pin|CS_LoRa_Pin|CSADC1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AMR_RESET_Pin|RST_LoRa_Pin|RX_Pin|TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CSADC3_Pin|CSADC2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CSAG_Pin AMR_RESET_Pin CS_LoRa_Pin RST_LoRa_Pin
                           RX_Pin TX_Pin CSADC1_Pin */
  GPIO_InitStruct.Pin = CSAG_Pin|AMR_RESET_Pin|CS_LoRa_Pin|RST_LoRa_Pin
                          |RX_Pin|TX_Pin|CSADC1_Pin;
>>>>>>>> f46711b7818af0db365b4cb3413287fc9408f5cf:AMR-CPU/Core/Src/main.c
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

<<<<<<<< HEAD:AMR_receiver/receiver/Core/Src/main.c
  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);
========
  /*Configure GPIO pins : DEN_Pin PB6 */
  GPIO_InitStruct.Pin = DEN_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CSADC3_Pin CSADC2_Pin */
  GPIO_InitStruct.Pin = CSADC3_Pin|CSADC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
>>>>>>>> f46711b7818af0db365b4cb3413287fc9408f5cf:AMR-CPU/Core/Src/main.c

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float read_battery(ADC_HandleTypeDef *hadc){
	uint16_t adc_read;
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc,HAL_MAX_DELAY);
	adc_read = HAL_ADC_GetValue(hadc);

	return (float)(adc_read*3.3)/4096;
}

void transmit_data(SPI_HandleTypeDef* spi, char* data){
	  uint8_t addr = RegIrqFlags;
	  LoRa_set_mode(spi,stdby_mode);
	  HAL_Delay(10);

	  LoRa_fill_fifo(spi, data, strlen(data));
	  LoRa_set_mode(spi, tx_mode);
	  HAL_Delay(10);
	  if(LoRa_read_reg(spi, addr) & 0x08){
		  LoRa_write_reg(spi,addr,0x08);
		  HAL_Delay(10);
	  }
	  HAL_Delay(10);
	  LoRa_set_mode(spi,sleep_mode);
	  HAL_Delay(10);
}

void get_measurements(SPI_HandleTypeDef* spi, adc_t adcx, adc_t adcy, adc_t adcz, char* data){
	  double tilt = 0;
	  float magx, magy, magz = 0.0;
	  float vol1, vol2, vol3;
	  uint16_t adcval1, adcval2, adcval3;

	  tilt = get_tilt(spi);

	  magx = get_magx(spi, adcx);
	  magy = get_magy(spi,adcy);
	  magz = get_magz(spi,adcz);

	  adcval1 = LTC2452_Read(spi, adcx);
	  adcval2 = LTC2452_Read(spi, adcy);
	  adcval3 = LTC2452_Read(spi, adcz);

	  vol1 = convVol(adcval1,3.3);
	  vol2 = convVol2(adcval2,3.3);
	  vol3 = convVol2(adcval3,3.3);

	  sprintf(data,"Volx: %4.3f\n\rX: %4.3f %cT\n\rVoly: %4.3f\n\rY: %4.3f %cT\n\rVolz: %4.3f\n\rZ: %4.3f %cT\n\rTilt: %4.2f%c",vol1,magx,0xE6,vol2, magy,0xE6, vol3,magz,0xE6,tilt, 0xA7);
}

void clrscr() {
	uart_buf_len = sprintf(uart_buf, "%c[2J", ESC);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}

void gotoxy(uint8_t x, uint8_t y) {
	uart_buf_len = sprintf(uart_buf, "%c[%d;%dH", ESC, y + 1, x + 1);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}

void uart_read(void) {
	uint8_t rx_data;
	uint8_t index = 0;
	while (1) {
		if (HAL_UART_Receive(&huart2, &rx_data, 1, HAL_MAX_DELAY) == HAL_OK) {
			if (rx_data == '\r') {
				rx_buffer[index] = '\0';
				index = 0;
				break;
			}
			else if (index < 256 - 1) {
				rx_buffer[index++] = rx_data;
			}
		}
	}
}

void enter_lpr_mode() {
	PWR -> CR1 |= 0x4000;
}

void exit_lpr_mode() {
	uint8_t x;
	PWR -> CR1 |= ~(0x4000);
	while (1) {
		x = (PWR -> SR2 & 0x200) >> 9;
		if (!x) {
			break;
		}
	}
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
