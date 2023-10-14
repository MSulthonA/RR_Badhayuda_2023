/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#define LCD_D4_Pin GPIO_PIN_3
#define LCD_D4_GPIO_Port GPIOE
#define LCD_D5_Pin GPIO_PIN_4
#define LCD_D5_GPIO_Port GPIOE
#define LCD_D6_Pin GPIO_PIN_5
#define LCD_D6_GPIO_Port GPIOE
#define LCD_D7_Pin GPIO_PIN_6
#define LCD_D7_GPIO_Port GPIOE
#define LED_BUILTIN_Pin GPIO_PIN_13
#define LED_BUILTIN_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_2
#define BUTTON_GPIO_Port GPIOA
#define SERVO0_Pin GPIO_PIN_10
#define SERVO0_GPIO_Port GPIOE
#define SERVO1_Pin GPIO_PIN_11
#define SERVO1_GPIO_Port GPIOE
#define SERVO2_Pin GPIO_PIN_12
#define SERVO2_GPIO_Port GPIOE
#define SERVO3_Pin GPIO_PIN_13
#define SERVO3_GPIO_Port GPIOE
#define SERVO4_Pin GPIO_PIN_14
#define SERVO4_GPIO_Port GPIOE
#define SERVO5_Pin GPIO_PIN_15
#define SERVO5_GPIO_Port GPIOE
#define SERVO6_Pin GPIO_PIN_11
#define SERVO6_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SERVO7_Pin GPIO_PIN_15
#define SERVO7_GPIO_Port GPIOA
#define LCD_RW_Pin GPIO_PIN_3
#define LCD_RW_GPIO_Port GPIOD
#define LCD_RS_Pin GPIO_PIN_4
#define LCD_RS_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOD
#define LCD_BL_Pin GPIO_PIN_3
#define LCD_BL_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_4
#define LCD_RST_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_7
#define LCD_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
