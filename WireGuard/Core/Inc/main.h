/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define RF_RESET_Pin GPIO_PIN_10
#define RF_RESET_GPIO_Port GPIOG
#define RF_SCK_Pin GPIO_PIN_0
#define RF_SCK_GPIO_Port GPIOK
#define RF_NSS_Pin GPIO_PIN_1
#define RF_NSS_GPIO_Port GPIOK
#define AI_01_Pin GPIO_PIN_7
#define AI_01_GPIO_Port GPIOF
#define RF_MISO_Pin GPIO_PIN_11
#define RF_MISO_GPIO_Port GPIOJ
#define AI_02_Pin GPIO_PIN_9
#define AI_02_GPIO_Port GPIOF
#define RF_MOSI_Pin GPIO_PIN_10
#define RF_MOSI_GPIO_Port GPIOJ
#define AI_03_Pin GPIO_PIN_2
#define AI_03_GPIO_Port GPIOC
#define AI_04_Pin GPIO_PIN_3
#define AI_04_GPIO_Port GPIOC
#define AI_07_Pin GPIO_PIN_6
#define AI_07_GPIO_Port GPIOA
#define AI_05_Pin GPIO_PIN_0
#define AI_05_GPIO_Port GPIOA
#define AI_06_Pin GPIO_PIN_1
#define AI_06_GPIO_Port GPIOA
#define AI_08_Pin GPIO_PIN_1
#define AI_08_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
