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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdio.h"
#include "string.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEDR_Pin GPIO_PIN_4
#define LEDR_GPIO_Port GPIOC
#define LEDG_Pin GPIO_PIN_5
#define LEDG_GPIO_Port GPIOC
#define LEDB_Pin GPIO_PIN_0
#define LEDB_GPIO_Port GPIOB
#define PWMR_Pin GPIO_PIN_6
#define PWMR_GPIO_Port GPIOC
#define DOWNR_Pin GPIO_PIN_7
#define DOWNR_GPIO_Port GPIOC
#define UPR_Pin GPIO_PIN_8
#define UPR_GPIO_Port GPIOC
#define PWML_Pin GPIO_PIN_9
#define PWML_GPIO_Port GPIOC
#define DOWNL_Pin GPIO_PIN_8
#define DOWNL_GPIO_Port GPIOA
#define UPL_Pin GPIO_PIN_9
#define UPL_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
extern char UARTgetchar[1],UARTbuffer[20];
extern uint8_t newblockdata;

extern uint32_t globaldata;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
