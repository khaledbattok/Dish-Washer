/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define salt_Pin GPIO_PIN_1
#define salt_GPIO_Port GPIOA
#define rins_Pin GPIO_PIN_2
#define rins_GPIO_Port GPIOA
#define flow_meter_Pin GPIO_PIN_3
#define flow_meter_GPIO_Port GPIOA
#define w_l_Pin GPIO_PIN_4
#define w_l_GPIO_Port GPIOA
#define door_Pin GPIO_PIN_5
#define door_GPIO_Port GPIOA
#define door_EXTI_IRQn EXTI9_5_IRQn
#define o_f_Pin GPIO_PIN_6
#define o_f_GPIO_Port GPIOA
#define o_f_EXTI_IRQn EXTI9_5_IRQn
#define dv_Pin GPIO_PIN_7
#define dv_GPIO_Port GPIOA
#define sv_Pin GPIO_PIN_0
#define sv_GPIO_Port GPIOB
#define iv_Pin GPIO_PIN_1
#define iv_GPIO_Port GPIOB
#define dp_Pin GPIO_PIN_2
#define dp_GPIO_Port GPIOB
#define wp_Pin GPIO_PIN_10
#define wp_GPIO_Port GPIOB
#define heat_relay_Pin GPIO_PIN_11
#define heat_relay_GPIO_Port GPIOB
#define power_flag_Pin GPIO_PIN_12
#define power_flag_GPIO_Port GPIOB
#define heart_Pin GPIO_PIN_13
#define heart_GPIO_Port GPIOB
#define buzzer_Pin GPIO_PIN_14
#define buzzer_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
