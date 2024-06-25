/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
<<<<<<<< HEAD:AMR_receiver/receiver/Core/Inc/main.h
#define CS_LoRa_Pin GPIO_PIN_4
#define CS_LoRa_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_8
#define TX_GPIO_Port GPIOA
#define RST_LoRa_Pin GPIO_PIN_9
#define RST_LoRa_GPIO_Port GPIOA
#define DIO0_Pin GPIO_PIN_10
#define DIO0_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_12
#define RX_GPIO_Port GPIOA
========
#define CSAG_Pin GPIO_PIN_1
#define CSAG_GPIO_Port GPIOA
#define AMR_RESET_Pin GPIO_PIN_2
#define AMR_RESET_GPIO_Port GPIOA
#define CS_LoRa_Pin GPIO_PIN_4
#define CS_LoRa_GPIO_Port GPIOA
#define DEN_Pin GPIO_PIN_0
#define DEN_GPIO_Port GPIOB
#define RST_LoRa_Pin GPIO_PIN_8
#define RST_LoRa_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_9
#define RX_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_10
#define TX_GPIO_Port GPIOA
#define CSADC1_Pin GPIO_PIN_11
#define CSADC1_GPIO_Port GPIOA
#define CSADC3_Pin GPIO_PIN_4
#define CSADC3_GPIO_Port GPIOB
#define CSADC2_Pin GPIO_PIN_5
#define CSADC2_GPIO_Port GPIOB
>>>>>>>> f46711b7818af0db365b4cb3413287fc9408f5cf:AMR-CPU/Core/Inc/main.h

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
