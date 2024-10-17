/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "my_function.h"
#include "stdio.h"
#include <rtthread.h>
#include <rthw.h>
#include "thread_manager.h"
#define old 0
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
#define Encoder_1_Pin GPIO_PIN_13
#define Encoder_1_GPIO_Port GPIOC
#define SERVO_LEFT_Pin GPIO_PIN_0
#define SERVO_LEFT_GPIO_Port GPIOA
#define SERVO_RIGHT_Pin GPIO_PIN_1
#define SERVO_RIGHT_GPIO_Port GPIOA
#define MS5837_SCL_Pin GPIO_PIN_0
#define MS5837_SCL_GPIO_Port GPIOB
#define MS5837_SDA_Pin GPIO_PIN_1
#define MS5837_SDA_GPIO_Port GPIOB
#define led_3_Pin GPIO_PIN_15
#define led_3_GPIO_Port GPIOA
#define SCL_1_Pin GPIO_PIN_4
#define SCL_1_GPIO_Port GPIOB
#define SDA_1_Pin GPIO_PIN_5
#define SDA_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
