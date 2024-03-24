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
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*  Choose your TRX----------------*/
#define G90
//#define TX500
/*---------------------------------*/

void Send_String_To_PC (char* str);
void Send_String_To_TRX (char* str);

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
#define LED_Pin LL_GPIO_PIN_15
#define LED_GPIO_Port GPIOC
#define OUT_160_Pin LL_GPIO_PIN_4
#define OUT_160_GPIO_Port GPIOA
#define OUT_80_Pin LL_GPIO_PIN_5
#define OUT_80_GPIO_Port GPIOA
#define OUT_40_Pin LL_GPIO_PIN_6
#define OUT_40_GPIO_Port GPIOA
#define OUT_30_20_Pin LL_GPIO_PIN_7
#define OUT_30_20_GPIO_Port GPIOA
#define OUT_17_15_Pin LL_GPIO_PIN_8
#define OUT_17_15_GPIO_Port GPIOA
#define OUT_12_10_Pin LL_GPIO_PIN_11
#define OUT_12_10_GPIO_Port GPIOA
#define OUT_6_Pin LL_GPIO_PIN_12
#define OUT_6_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
