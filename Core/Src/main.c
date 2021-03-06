/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi3;

/* Definitions for bmeSampleTask */
osThreadId_t bmeSampleTaskHandle;
const osThreadAttr_t bmeSampleTask_attributes = {
  .name = "bmeSampleTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for lcdRenderTask */
osThreadId_t lcdRenderTaskHandle;
const osThreadAttr_t lcdRenderTask_attributes = {
  .name = "lcdRenderTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for tapCheckTask */
osThreadId_t tapCheckTaskHandle;
const osThreadAttr_t tapCheckTask_attributes = {
  .name = "tapCheckTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for sdWriteTask */
osThreadId_t sdWriteTaskHandle;
const osThreadAttr_t sdWriteTask_attributes = {
  .name = "sdWriteTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* USER CODE BEGIN PV */
volatile uint32_t* DWT_CTRL = (uint32_t*)0xE0001000;

enum eBmeState { eStream, eRecord, eWrite };
enum eBmeState curr_state = eStream;

__IO FlagStatus TouchDetected     = RESET;
FlagStatus LcdInitialized = RESET;
FlagStatus TsInitialized  = RESET;

extern struct bme280_dev bme_device;
extern struct bme280_data curr_bme_data;

struct sd_write_item {
	struct bme280_data readings;
	uint32_t tick_offset;
};
struct sd_write_item sd_raw_data_buf[NUMBER_OF_SAMPLES_TO_REC];
int iSdRawDataBuf = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_ICACHE_Init(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
void BmeSampleTask(void *argument);
void LcdRenderTask(void *argument);
void TapCheckTask(void *argument);
void SdWriteTask(void *argument);

/* USER CODE BEGIN PFP */
void LcdRenderReading(const struct bme280_data *reading);
UINT WriteReadingAndCommaToFile(FIL* SDFile, char* single_line_buf, int bufSize);
static void SystemHardwareInit(void);
extern void SEGGER_UART_init(U32 baud);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  *(DWT_CTRL) |= (1 << 0);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_ICACHE_Init();
  MX_GPIO_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  SystemHardwareInit();

  /* Initialize the micro SD Card */
  if (BSP_SD_Init(0) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  if (BSP_SD_DetectITConfig(0) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }

  // Print start screen to LCD display
  UTIL_LCD_Clear(LCD_BACKGROUND_COLOR);
  UTIL_LCD_FillRect(0, 0, 240, 24, UTIL_LCD_COLOR_ST_BLUE_DARK);
  UTIL_LCD_SetTextColor(LCD_TEXT_COLOR);
  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_ST_BLUE_DARK);
  UTIL_LCD_SetFont(&Font24);
  UTIL_LCD_DisplayStringAt(0, 0, (uint8_t *)"THP Sensor", CENTER_MODE);
  UTIL_LCD_SetBackColor(LCD_BACKGROUND_COLOR);
  UTIL_LCD_SetFont(&Font12);
  UTIL_LCD_DisplayStringAt(0, PY_TEMPERATURE, (uint8_t *)"Temperature (C): ", LEFT_MODE);
  UTIL_LCD_DisplayStringAt(0, PY_HUMIDITY, (uint8_t *)"Humidity (%RH): ", LEFT_MODE);
  UTIL_LCD_DisplayStringAt(0, PY_PRESSURE, (uint8_t *)"Pressure (hPa): ", LEFT_MODE);
  UTIL_LCD_FillRect(0, 132, 240, 108, UTIL_LCD_COLOR_ST_BLUE_DARK);
  UTIL_LCD_FillRect(PX_RECORD_BUTTON, PY_RECORD_BUTTON, WIDTH_RECORD_BUTTON, HEIGHT_RECORD_BUTTON, RECORD_BUTTON_COLOR_UNPR);
  UTIL_LCD_SetBackColor(RECORD_BUTTON_COLOR_UNPR);
  UTIL_LCD_DisplayStringAt(0, PY_RECORD_BUTTON_TEXT, (uint8_t *)BUTTON_TEXT_STREAMING, CENTER_MODE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of bmeSampleTask */
  bmeSampleTaskHandle = osThreadNew(BmeSampleTask, NULL, &bmeSampleTask_attributes);

  /* creation of lcdRenderTask */
  lcdRenderTaskHandle = osThreadNew(LcdRenderTask, NULL, &lcdRenderTask_attributes);

  /* creation of tapCheckTask */
  tapCheckTaskHandle = osThreadNew(TapCheckTask, NULL, &tapCheckTask_attributes);

  /* creation of sdWriteTask */
  sdWriteTaskHandle = osThreadNew(SdWriteTask, NULL, &sdWriteTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadSuspend(sdWriteTaskHandle);
//  SEGGER_UART_init(SEGGER_UART_BAUD_RATE);
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */
  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void SystemHardwareInit(void)
{
  /* Initialize the LCD */
  if (LcdInitialized != SET)
  {
    LCD_UTILS_Drv_t lcdDrv;

    /* Initialize the LCD */
    if (BSP_LCD_Init(0, LCD_ORIENTATION_PORTRAIT) != BSP_ERROR_NONE)
    {
      Error_Handler();
    }

    /* Set UTIL_LCD functions */
    lcdDrv.DrawBitmap  = BSP_LCD_DrawBitmap;
    lcdDrv.FillRGBRect = BSP_LCD_FillRGBRect;
    lcdDrv.DrawHLine   = BSP_LCD_DrawHLine;
    lcdDrv.DrawVLine   = BSP_LCD_DrawVLine;
    lcdDrv.FillRect    = BSP_LCD_FillRect;
    lcdDrv.GetPixel    = BSP_LCD_ReadPixel;
    lcdDrv.SetPixel    = BSP_LCD_WritePixel;
    lcdDrv.GetXSize    = BSP_LCD_GetXSize;
    lcdDrv.GetYSize    = BSP_LCD_GetYSize;
    lcdDrv.SetLayer    = BSP_LCD_SetActiveLayer;
    lcdDrv.GetFormat   = BSP_LCD_GetFormat;
    UTIL_LCD_SetFuncDriver(&lcdDrv);

    /* Clear the LCD */
    UTIL_LCD_Clear(LCD_BACKGROUND_COLOR);

    /* Set the display on */
    if (BSP_LCD_DisplayOn(0) != BSP_ERROR_NONE)
    {
      Error_Handler();
    }

    LcdInitialized = SET;
  }

  /* Initialize the TouchScreen */
  if (TsInitialized != SET)
  {
    TS_Init_t TsInit;

    /* Initialize the TouchScreen */
    TsInit.Width       = 240;
    TsInit.Height      = 240;
    TsInit.Orientation = TS_ORIENTATION_PORTRAIT;
    TsInit.Accuracy    = 10;
    if (BSP_TS_Init(0, &TsInit) != BSP_ERROR_NONE)
    {
      Error_Handler();
    }

    /* Configure TS interrupt */
    if (BSP_TS_EnableIT(0) != BSP_ERROR_NONE)
    {
      Error_Handler();
    }

    TsInitialized = SET;
  }
}

void LcdRenderReading(const struct bme280_data *reading) {
	static char sensor_reading_buf[14];
	static const uint clear_height = 12;
	static const uint clear_width = 98;

	UTIL_LCD_SetBackColor(LCD_BACKGROUND_COLOR);

	if (WriteTemperature(reading->temperature, sensor_reading_buf, sizeof(sensor_reading_buf)) > 0) {
		UTIL_LCD_FillRect(240 - clear_width, PY_TEMPERATURE, clear_width, clear_height, LCD_BACKGROUND_COLOR);
		UTIL_LCD_DisplayStringAt(0, PY_TEMPERATURE, (uint8_t *)sensor_reading_buf, RIGHT_MODE);
		memset(sensor_reading_buf, '\0', sizeof(sensor_reading_buf));
	}

	if (WriteHumidity(reading->humidity, sensor_reading_buf, sizeof(sensor_reading_buf))) {
		UTIL_LCD_FillRect(240 - clear_width, PY_HUMIDITY, clear_width, clear_height, LCD_BACKGROUND_COLOR);
		UTIL_LCD_DisplayStringAt(0, PY_HUMIDITY, (uint8_t *)sensor_reading_buf, RIGHT_MODE);
		memset(sensor_reading_buf, '\0', sizeof(sensor_reading_buf));
	}


	if (WritePressure(reading->pressure, sensor_reading_buf, sizeof(sensor_reading_buf)) > 0) {
		UTIL_LCD_FillRect(240 - clear_width, PY_PRESSURE, clear_width, clear_height, LCD_BACKGROUND_COLOR);
		UTIL_LCD_DisplayStringAt(0, PY_PRESSURE, (uint8_t *)sensor_reading_buf, RIGHT_MODE);
		memset(sensor_reading_buf, '\0', sizeof(sensor_reading_buf));
	}
}

UINT WriteReadingAndCommaToFile(FIL* SDFile, char* single_line_buf, int bufSize) {
	const char separator = ',';
	FRESULT res;
	UINT totbyteswritten = 0, byteswritten = 0;
	res = f_write(SDFile, single_line_buf, bufSize, (void*)&byteswritten);
	totbyteswritten += byteswritten;
	if (res != FR_OK || byteswritten != bufSize) return totbyteswritten;
	res = f_write(SDFile, &separator, sizeof(separator), (void*)&byteswritten);
	return (totbyteswritten += byteswritten);
}

void BSP_TS_Callback(uint32_t Instance) {
	if (Instance == 0) {
		// Signal tapCheck that new tap was detected
		TouchDetected = SET;
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_BmeSampleTask */
/**
  * @brief  Function implementing the bmeSampleTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_BmeSampleTask */
void BmeSampleTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	if (bme280_app_init() == BME280_OK) {
		uint32_t bme280_sample_delay = bme280_cal_meas_delay(&(bme_device.settings));
		TickType_t xLastWakeTime = osKernelGetTickCount();
		/* Infinite loop */
		for (;;) {
			if (bme280_set_sensor_mode(BME280_FORCED_MODE, &bme_device) == BME280_OK) {
				osDelay(pdMS_TO_TICKS(bme280_sample_delay));
				if (bme280_get_sensor_data(BME280_ALL, &curr_bme_data, &bme_device) == BME280_OK) {
					// Signal lcd render and sd write that new data is ready
					osThreadFlagsSet(lcdRenderTaskHandle, BME_FLAG_NEW_DATA);
					osThreadFlagsSet(sdWriteTaskHandle, BME_FLAG_NEW_DATA);
				}
			}
			// Wait for the next cycle
			xLastWakeTime += pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
			osDelayUntil(xLastWakeTime);
		}
	}
	osThreadExit();
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LcdRenderTask */
/**
* @brief Function implementing the lcdRenderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LcdRenderTask */
void LcdRenderTask(void *argument)
{
  /* USER CODE BEGIN LcdRenderTask */
	uint32_t lcd_render_flags;
	/* Infinite loop */
	for (;;) {
		// Check if there is something new to render (new data, user event)
		lcd_render_flags = osThreadFlagsWait(
				BME_FLAG_NEW_DATA | BME_FLAG_START_RECORDING | BME_FLAG_RECORDING_DONE | BME_FLAG_SD_WR_DONE,
				osFlagsWaitAny,
				osWaitForever);

		if (lcd_render_flags & BME_FLAG_NEW_DATA) {
			// Display curr readings
			LcdRenderReading(&curr_bme_data);
		}
		if (lcd_render_flags & BME_FLAG_START_RECORDING) {
			UTIL_LCD_FillRect(PX_RECORD_BUTTON, PY_RECORD_BUTTON, WIDTH_RECORD_BUTTON, HEIGHT_RECORD_BUTTON, RECORD_BUTTON_COLOR_PRSS);
			UTIL_LCD_SetBackColor(RECORD_BUTTON_COLOR_PRSS);
			UTIL_LCD_DisplayStringAt(0, PY_RECORD_BUTTON_TEXT, (uint8_t*)BUTTON_TEXT_RECORDING, CENTER_MODE);
			osThreadResume(sdWriteTaskHandle);
		}
		if (lcd_render_flags & BME_FLAG_RECORDING_DONE) {
			UTIL_LCD_FillRect(PX_RECORD_BUTTON, PY_RECORD_BUTTON, WIDTH_RECORD_BUTTON, HEIGHT_RECORD_BUTTON, RECORD_BUTTON_COLOR_PRSS);
			UTIL_LCD_SetBackColor(RECORD_BUTTON_COLOR_PRSS);
			UTIL_LCD_DisplayStringAt(0, PY_RECORD_BUTTON_TEXT, (uint8_t*)BUTTON_TEXT_WRITING, CENTER_MODE);
		}
		if (lcd_render_flags & BME_FLAG_SD_WR_DONE) {
		  UTIL_LCD_FillRect(PX_RECORD_BUTTON, PY_RECORD_BUTTON, WIDTH_RECORD_BUTTON, HEIGHT_RECORD_BUTTON, RECORD_BUTTON_COLOR_UNPR);
		  UTIL_LCD_SetBackColor(RECORD_BUTTON_COLOR_UNPR);
		  UTIL_LCD_DisplayStringAt(0, PY_RECORD_BUTTON_TEXT, (uint8_t *)BUTTON_TEXT_STREAMING, CENTER_MODE);
		  osThreadResume(tapCheckTaskHandle);
		}
	}
	osThreadExit();
  /* USER CODE END LcdRenderTask */
}

/* USER CODE BEGIN Header_TapCheckTask */
/**
* @brief Function implementing the tapCheckTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TapCheckTask */
void TapCheckTask(void *argument)
{
  /* USER CODE BEGIN TapCheckTask */
  uint16_t x, y;
  uint8_t shouldSuspend = 0;
	TS_MultiTouch_State_t TsMultipleState = {0};
  /* Infinite loop */
  for(;;)
  {
  	if (shouldSuspend) {
  		shouldSuspend = FALSE;
  		osThreadSuspend(tapCheckTaskHandle);
  	}
		if (TouchDetected == SET) {
			taskENTER_CRITICAL();
			TouchDetected = RESET;
			if (BSP_TS_Get_MultiTouchState(0, &TsMultipleState) != BSP_ERROR_NONE) Error_Handler();
			// Check if tap on button and state is stream
			if (TsMultipleState.TouchDetected >= 1) {
				x = TsMultipleState.TouchX[0];
				y = TsMultipleState.TouchY[0];
				if (x > PX_RECORD_BUTTON && x < PX_RECORD_BUTTON + WIDTH_RECORD_BUTTON &&
						y > PY_RECORD_BUTTON && y < PY_RECORD_BUTTON + HEIGHT_RECORD_BUTTON) {
						// Set render and sd write thread flags
						osThreadFlagsSet(lcdRenderTaskHandle, BME_FLAG_START_RECORDING);
						osThreadFlagsSet(sdWriteTaskHandle, BME_FLAG_START_RECORDING);
						shouldSuspend = TRUE;
				}
			}
			taskEXIT_CRITICAL();
		}
		osDelay(pdMS_TO_TICKS(TAP_POLL_PERIOD_MS));
	}
	osThreadExit();
  /* USER CODE END TapCheckTask */
}

/* USER CODE BEGIN Header_SdWriteTask */
/**
* @brief Function implementing the sdWriteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SdWriteTask */
void SdWriteTask(void *argument) {
	/* USER CODE BEGIN SdWriteTask */
	struct sd_write_item temp_write_item;
	uint32_t rec_tick = 0, last_tick = 0, curr_tick = 0;
	FRESULT res;
	FILINFO fInfo;
	uint32_t bytestowrite, byteswritten;
	const char lr = '\n';
	char single_line_buf[14];
	/* Infinite loop */
	for (;;) {
		if (iSdRawDataBuf < NUMBER_OF_SAMPLES_TO_REC) {
			// Wait for new data then write to sd buf
			osThreadFlagsWait(BME_FLAG_NEW_DATA, osFlagsWaitAny, osWaitForever);
			if (iSdRawDataBuf == 0) {
				rec_tick = osKernelGetTickCount();
				last_tick = rec_tick;
			} else {
				curr_tick = osKernelGetTickCount();
				rec_tick = (curr_tick > last_tick) ?
								(rec_tick + curr_tick - last_tick) :
								(UINT_MAX - last_tick + curr_tick);
				last_tick = curr_tick;
			}
			temp_write_item.tick_offset = rec_tick;
			temp_write_item.readings = curr_bme_data;
			sd_raw_data_buf[iSdRawDataBuf++] = temp_write_item;
		} else {
			osThreadFlagsSet(lcdRenderTaskHandle, BME_FLAG_RECORDING_DONE);
			// Register the file system object to the FatFs module
			if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) == FR_OK) {
				// If file exists already, delete it
				if (f_stat("THP_REC.TXT", &fInfo) == FR_OK) f_unlink("THP_REC.TXT");

				// Create and Open a new text file object with write access
				if (f_open(&SDFile, "THP_REC.TXT", FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
					for (int i = 0; i < iSdRawDataBuf; i++) {
						// Write tick
						bytestowrite = sprintf(single_line_buf, "%u,", (uint) sd_raw_data_buf[i].tick_offset);
						res = f_write(&SDFile, single_line_buf, bytestowrite, (void*) &byteswritten);
						if (byteswritten != bytestowrite) break;

						// Write temperature
						bytestowrite = WriteTemperature(sd_raw_data_buf[i].readings.temperature, single_line_buf, sizeof(single_line_buf));
						if (bytestowrite < 0) break;
						if (WriteReadingAndCommaToFile(&SDFile, single_line_buf, bytestowrite) != bytestowrite + 1) break;
						memset(single_line_buf, '\0', sizeof(single_line_buf));

						// Write humidity
						bytestowrite = WriteHumidity(sd_raw_data_buf[i].readings.humidity, single_line_buf, sizeof(single_line_buf));
						if (bytestowrite < 0) break;
						if (WriteReadingAndCommaToFile(&SDFile, single_line_buf, bytestowrite) != bytestowrite + 1) break;
						memset(single_line_buf, '\0', sizeof(single_line_buf));

						// Write pressure
						bytestowrite = WritePressure(sd_raw_data_buf[i].readings.pressure, single_line_buf, sizeof(single_line_buf));
						if (bytestowrite < 0) break;
						if (WriteReadingAndCommaToFile(&SDFile, single_line_buf, bytestowrite) != bytestowrite + 1) break;
						memset(single_line_buf, '\0', sizeof(single_line_buf));

						// Write new line
						res = f_write(&SDFile, &lr, sizeof(lr), (void*) &byteswritten);
						if (res != FR_OK || byteswritten != sizeof(lr)) break;
					}
					// Write new line
					res = f_write(&SDFile, &lr, sizeof(lr), (void*) &byteswritten);
					// Close the open text file
					f_close(&SDFile);
				}
				f_mount(0, (TCHAR const*) SDPath, 0);
			}
			iSdRawDataBuf = 0;
			memset(sd_raw_data_buf, '\0', sizeof(sd_raw_data_buf));

			// Raise wr done flag then suspend
			osThreadFlagsSet(lcdRenderTaskHandle, BME_FLAG_SD_WR_DONE);
			osThreadSuspend(sdWriteTaskHandle);
		}
	}
	osThreadExit();
	/* USER CODE END SdWriteTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
