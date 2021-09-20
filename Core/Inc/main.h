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
#include "stm32l5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "stm32l562e_discovery.h"
#include "stm32l562e_discovery_lcd.h"
#include "stm32l562e_discovery_ts.h"
#include "stm32l562e_discovery_sd.h"
#include "app_fatfs.h"
#include "bme280.h"
#include "bme280_app.h"
#include "stm32_lcd.h"
#include "task.h"
#include "write_readings.h"
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
/* USER CODE BEGIN Private defines */
#define LCD_BACKGROUND_COLOR 			UTIL_LCD_COLOR_BLACK
#define LCD_TEXT_COLOR						UTIL_LCD_COLOR_WHITE
#define PY_TEMPERATURE						50
#define PY_HUMIDITY								80
#define PY_PRESSURE								110
#define	RECORD_BUTTON_COLOR_UNPR	UTIL_LCD_COLOR_ST_GRAY_DARK
#define	RECORD_BUTTON_COLOR_PRSS	0xFF222222UL
#define PX_RECORD_BUTTON					20
#define PY_RECORD_BUTTON					144
#define PY_RECORD_BUTTON_TEXT			(PY_RECORD_BUTTON + (HEIGHT_RECORD_BUTTON >> 1) - 6)
#define WIDTH_RECORD_BUTTON				(240 - PX_RECORD_BUTTON - PX_RECORD_BUTTON)
#define HEIGHT_RECORD_BUTTON			84
#define	BUTTON_TEXT_STREAMING			"Record"
#define	BUTTON_TEXT_RECORDING			"Recording..."
#define	BUTTON_TEXT_WRITING				"Writing to card..."
#define SAMPLE_PERIOD_MS 					500
#define NUMBER_OF_SAMPLES_TO_REC	15
#define TAP_POLL_PERIOD_MS				20
#define BME_FLAG_NEW_DATA					( 0x01 << 0 )
#define BME_FLAG_START_RECORDING	( 0x01 << 1 )
#define BME_FLAG_RECORDING_DONE		( 0x01 << 2 )
#define BME_FLAG_SD_WR_DONE				( 0x01 << 3 )
#define SEGGER_UART_BAUD_RATE			28000
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
