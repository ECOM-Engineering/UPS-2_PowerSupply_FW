/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_dma.h"

#include "stm32g0xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UPS-2.h"
#include "lowlevel.h"
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
void Error_Handler(void);
//void ecTxString(char *String, size_t Size);
//void USART_Rx_Callback(void);
//void ecUSART_Rx_Error_Callback(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pi_Pin GPIO_PIN_7
#define LED_Pi_GPIO_Port GPIOB
#define LED_Batt_Pin GPIO_PIN_9
#define LED_Batt_GPIO_Port GPIOB
#define LED_Main_Pin GPIO_PIN_15
#define LED_Main_GPIO_Port GPIOC
#define EN_5V_Pin GPIO_PIN_0
#define EN_5V_GPIO_Port GPIOA
#define ADC_VIN_MAIN_Pin GPIO_PIN_1
#define ADC_VIN_MAIN_GPIO_Port GPIOA
#define ADC_VIN_BATT_Pin GPIO_PIN_4
#define ADC_VIN_BATT_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define ADC_5V_PI_Pin GPIO_PIN_6
#define ADC_5V_PI_GPIO_Port GPIOA
#define nBatt_Pin GPIO_PIN_7
#define nBatt_GPIO_Port GPIOA
#define Mode_Pin GPIO_PIN_8
#define Mode_GPIO_Port GPIOA
#define CMD_OUT_Pin GPIO_PIN_11
#define CMD_OUT_GPIO_Port GPIOA
#define ACK_IN_Pin GPIO_PIN_12
#define ACK_IN_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_6
#define BUTTON_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
