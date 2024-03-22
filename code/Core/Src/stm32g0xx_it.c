/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern int flag_band;
extern int time_last_pcdata;
extern int isDataReady;
extern char PCData[40];
extern char TRXData[40];


/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
	if (LL_TIM_IsActiveFlag_UPDATE(TIM14)) {
		LL_TIM_ClearFlag_UPDATE(TIM14);
		LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
  /* USER CODE END TIM14_IRQn 0 */
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
#ifdef TX500
	char letter;
	static uint8_t cnt = 0;
	if (LL_USART_IsActiveFlag_RXNE(USART1)) {
		LL_USART_DisableIT_RXNE(USART2);
		letter = LL_USART_ReceiveData8(USART1);
		LL_USART_TransmitData8(USART2, letter);
		if (letter != ';') {
			PCData[cnt] = letter;
			cnt++;
			if (cnt == 39)
				cnt = 0;
		} else {
			PCData[cnt] = ';';
			cnt = 0;
			if ((PCData[0] == 'I') && (PCData[1] == 'F')) {
				time_last_pcdata = HAL_GetTick();
			}
		}
		LL_USART_EnableIT_RXNE(USART2);
	}
#endif

#ifdef G90
	char letter;
		static uint8_t cnt = 0;
		if (LL_USART_IsActiveFlag_RXNE(USART1)) {
			LL_USART_DisableIT_RXNE(USART2);
			letter = LL_USART_ReceiveData8(USART1);
			LL_USART_TransmitData8(USART2, letter);
			if (letter != 0xFD) {
				PCData[cnt] = letter;
				cnt++;
				if (cnt == 15)
					cnt = 0;
			} else {
				PCData[cnt] = 0xFD;
				cnt = 0;
				if ((PCData[0] == 0xFE) && (PCData[1] == 0xFE)) {
					time_last_pcdata = HAL_GetTick();
				}
			}
			LL_USART_EnableIT_RXNE(USART2);
		}
#endif
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
#ifdef TX500
	char letter;
	static uint8_t i = 0;
	if (LL_USART_IsActiveFlag_RXNE(USART2)) {
		LL_USART_DisableIT_RXNE(USART1);
		letter = LL_USART_ReceiveData8(USART2);
		LL_USART_TransmitData8(USART1, letter);
		if (letter != ';') {
			TRXData[i] = letter;
			i++;
			isDataReady = 0;
			if (i == 39)
				i = 0;
		} else {
			TRXData[i] = ';';
			i = 0;
			if ((TRXData[0] == 'I') && (TRXData[1] == 'F')) {
				isDataReady = 1;
				if ((TRXData[5] == '0') && (TRXData[6] == '1')
						&& ((TRXData[7] == '8') || (TRXData[7] == '9')))
					flag_band = 160; //160m 1800-1900 kHz
				if ((TRXData[5] == '0') && (TRXData[6] == '3'))
					flag_band = 80; //80m 3500-3800 kHz
				if ((TRXData[5] == '0') && (TRXData[6] == '5')
						&& ((TRXData[7] == '3') || (TRXData[7] == '4')))
					flag_band = 60; //60m 5300-5400 kHz
				if ((TRXData[5] == '0') && (TRXData[6] == '7')
						&& ((TRXData[7] == '0') || (TRXData[7] == '1')
								|| (TRXData[7] == '2')))
					flag_band = 40; //40m 7000-7200 kHz
				if ((TRXData[5] == '1') && (TRXData[6] == '0')
						&& (TRXData[7] == '1'))
					flag_band = 30; //30m 10100 kHz
				if ((TRXData[5] == '1') && (TRXData[6] == '4'))
					flag_band = 20; //20m 14000-14350 kHz
				if ((TRXData[5] == '1') && (TRXData[6] == '8')
						&& ((TRXData[7] == '0') || (TRXData[7] == '1')))
					flag_band = 17; //17m 18068-18150 kHz
				if ((TRXData[5] == '2') && (TRXData[6] == '1'))
					flag_band = 15; //15m 21000-21450 kHz
				if ((TRXData[5] == '2') && (TRXData[6] == '4')
						&& ((TRXData[7] == '8') || (TRXData[7] == '9')))
					flag_band = 12; //12m 24890-24950 kHz
				if ((TRXData[5] == '2')
						&& ((TRXData[6] == '8') || (TRXData[6] == '9')))
					flag_band = 10; //10m 28000-29900 kHz
				if ((TRXData[5] == '5') && (TRXData[6] == '0'))
					flag_band = 6; //6m 50100-50400 kHz
			}

		}
		LL_USART_EnableIT_RXNE(USART1);

	}
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
#endif

#ifdef G90
	uint8_t letter;
		static uint8_t i = 0;
		if (LL_USART_IsActiveFlag_RXNE(USART2)) {
			LL_USART_DisableIT_RXNE(USART1);
			letter = LL_USART_ReceiveData8(USART2);
			LL_USART_TransmitData8(USART1, letter);
			if (letter != 0xFD) {
				TRXData[i] = letter;
				i++;
				isDataReady = 0;
				if (i == 15)
					i = 0;
			} else {
				TRXData[i] = 0xFD;
				i = 0;
				if ((TRXData[0] == 0xFE) && (TRXData[1] == 0xFE)
						&& ((TRXData[4] == 0x00) || (TRXData[4] == 0x03))) {
					isDataReady = 1;
					if ((TRXData[8] == 0x01) || (TRXData[8] == 0x02))
						flag_band = 160; //160m 1000-2999 kHz
					if ((TRXData[8] == 0x03) || (TRXData[8] == 0x04))
						flag_band = 80; //80m 3000-4999 kHz
					if ((TRXData[8] == 0x05))
						flag_band = 60; //60m 5000-5999 kHz
					if ((TRXData[8] == 0x07))
						flag_band = 40; //40m 7000-7999 kHz
					if ((TRXData[8] == 0x10))
						flag_band = 30; //30m 10000-10999 kHz
					if ((TRXData[8] == 0x14))
						flag_band = 20; //20m 14000-14999 kHz
					if ((TRXData[8] == 0x18))
						flag_band = 17; //17m 18000-18999 kHz
					if ((TRXData[8] == 0x21))
						flag_band = 15; //15m 21000-21999 kHz
					if ((TRXData[8] == 0x24))
						flag_band = 12; //12m 24000-24999 kHz
					if ((TRXData[8] == 0x28) || (TRXData[8] == 0x29))
						flag_band = 10; //10m 28000-29999 kHz
					if ((TRXData[8] == 0x050))
						flag_band = 6; //6m 50000-50999 kHz
				}

			}
			LL_USART_EnableIT_RXNE(USART1);
		}
#endif
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
