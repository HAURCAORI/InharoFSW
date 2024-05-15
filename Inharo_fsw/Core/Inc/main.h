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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "logger.h"
#include "converter.h"

#include "initialize/adc.h"
#include "initialize/clock.h"
#include "initialize/gpio.h"
#include "initialize/i2c.h"
#include "initialize/rtc.h"
#include "initialize/spi.h"
#include "initialize/tim.h"
#include "initialize/usart.h"
#include "initialize/dma.h"

#include "module/buzzer.h"
#include "module/bmp390/bmp390_stm32.h"
#include "module/bno055/bno055_stm32.h"
#include "module/fatfs_sd/fatfs_sd.h"
#include "module/gps/gps.h"
#include <module/mpxv7002dp/mpxv7002dp.h>
#include "module/servo/servo.h"
#include "module/xbee/xbee.h"


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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define BUZ_Pin GPIO_PIN_1
#define BUZ_GPIO_Port GPIOC
#define CM1_Pin GPIO_PIN_2
#define CM1_GPIO_Port GPIOC
#define CM2_Pin GPIO_PIN_3
#define CM2_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB
#define USB_IO_Pin GPIO_PIN_10
#define USB_IO_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
