/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "type.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IH_UART1_MAX_LENGTH 80
#define IH_UART1_HEADER1 'G'
#define IH_UART1_HEADER2 'G'
#define IH_UART1_HEADER3 'A'
#define IH_UART1_TERMINATOR '*'

#define GPS_MAX_LENGTH 82


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Definitions for Main */
osThreadId_t MainHandle;
const osThreadAttr_t Main_attributes = {
  .name = "Main",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for GPS */
osThreadId_t GPSHandle;
const osThreadAttr_t GPS_attributes = {
  .name = "GPS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StateManaging */
osThreadId_t StateManagingHandle;
const osThreadAttr_t StateManaging_attributes = {
  .name = "StateManaging",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Receive */
osThreadId_t ReceiveHandle;
const osThreadAttr_t Receive_attributes = {
  .name = "Receive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Debug */
osThreadId_t DebugHandle;
const osThreadAttr_t Debug_attributes = {
  .name = "Debug",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorReading */
osTimerId_t SensorReadingHandle;
const osTimerAttr_t SensorReading_attributes = {
  .name = "SensorReading"
};
/* Definitions for Transmit */
osTimerId_t TransmitHandle;
const osTimerAttr_t Transmit_attributes = {
  .name = "Transmit"
};

/* Definitions for TransmitSemaphore */
osSemaphoreId_t ReceiveSemaphoreHandle;
const osSemaphoreAttr_t ReceiveSemaphore_attributes = {
  .name = "ReceiveSemaphore"
};

// ----------EVENT-------------

/* Definitions for GPSEvent */
osEventFlagsId_t EventGPSHandle;
const osEventFlagsAttr_t EventGPS_attributes = {
  .name = "EventGPS"
};
/* Definitions for CommandEvent */
osEventFlagsId_t EventCommandHandle;
const osEventFlagsAttr_t EventCommand_attributes = {
  .name = "EventCommand"
};

/* Definitions for CommandEvent */
osEventFlagsId_t EventReceiveHandle;
const osEventFlagsAttr_t EventReceive_attributes = {
  .name = "EventReceive"
};
/* USER CODE BEGIN PV */

Servo_HandleTypeDef hservo1, hservo2, hservo3;
USB_Buffer_Type usb_rx_buffer;
volatile Sensor_Data sensor_data;

uint8_t IH_UART1_headerPass = 0;
uint8_t IH_UART1_pMessage = 0;
uint8_t IH_UART1_byteBuf = 0;
uint8_t IH_UART1_buf[IH_UART1_MAX_LENGTH] ={0,};

uint8_t GPS_message[GPS_MAX_LENGTH] = {0,};
uint8_t GPS_notReadFlag = 1;
uint8_t GPS_overwriteFlag = 1;
uint8_t GPS_onceOverwriteFlag = 1;

// XBee 통신 관련 정의
uint8_t uart_rx_buffer[1];
uint32_t receive_tick;
XBEE_Buffer_Type xbee_rx_buffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void vMainTask(void *argument);
void vGPSTask(void *argument); 						// GPS communication cycle
void vStateManagingTask(void *argument);	// state managing cycle
void vReceiveTask(void *argument);				// receive cycle
void vDebugTask(void *argument);
void vSensorReadingCallback(void *argument);
void vTransmitCallback(void *argument);

/* USER CODE BEGIN PFP */
static void _BMP390_Init(void);
static void _BNO055_Init(void);
static void _SD_Init(void);
static void _Servo_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  logi("Initializing...");

  _BMP390_Init();
  _BNO055_Init();
  _SD_Init();
  _Servo_Init();

  logi("Initialized");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of TransmitSemaphore */
  ReceiveSemaphoreHandle = osSemaphoreNew(1, 1, &ReceiveSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of SensorReading */
  SensorReadingHandle = osTimerNew(vSensorReadingCallback, osTimerPeriodic, NULL, &SensorReading_attributes);

  /* creation of Transmit */
  TransmitHandle = osTimerNew(vTransmitCallback, osTimerPeriodic, NULL, &Transmit_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(SensorReadingHandle, 3000);
  osTimerStart(TransmitHandle, 3000);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Main */
  MainHandle = osThreadNew(vMainTask, NULL, &Main_attributes);

  /* creation of GPS */
  GPSHandle = osThreadNew(vGPSTask, NULL, &GPS_attributes);

  /* creation of StateManaging */
  StateManagingHandle = osThreadNew(vStateManagingTask, NULL, &StateManaging_attributes);

  /* creation of Receive */
  ReceiveHandle = osThreadNew(vReceiveTask, NULL, &Receive_attributes);

  /* creation of Debug */
  DebugHandle = osThreadNew(vDebugTask, NULL, &Debug_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of GPSEvent */
  EventGPSHandle = osEventFlagsNew(&EventGPS_attributes);

  /* creation of CommandEvent */
  EventCommandHandle = osEventFlagsNew(&EventCommand_attributes);

  EventReceiveHandle = osEventFlagsNew(&EventReceive_attributes);
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

/* USER CODE BEGIN 4 */
//uint8_t buffer[14] = {};



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
  	//osSemaphoreRelease(ReceiveSemaphoreHandle);

  	if(HAL_GetTick() - receive_tick > 10) {
  		xbee_rx_buffer.isReceiving = FALSE;
  		receive_tick = HAL_GetTick();
  	}

  	if(xbee_rx_buffer.isReceiving == FALSE) {
  		if(uart_rx_buffer[0] == 0x7E){
  			memset(&xbee_rx_buffer,0, sizeof(xbee_rx_buffer));
  			xbee_rx_buffer.isReceiving = TRUE;
  	  	++xbee_rx_buffer.pos;
  		}
    	HAL_UART_Receive_IT(&huart3, uart_rx_buffer, sizeof(uart_rx_buffer));
  		return;
  	}

  	if(xbee_rx_buffer.pos == 0) {
  		xbee_rx_buffer.length = 0;
  	} else if(xbee_rx_buffer.pos == 1) {
  		xbee_rx_buffer.length = uart_rx_buffer[0] * 0xFF;
  	} else if(xbee_rx_buffer.pos == 2) {
  		xbee_rx_buffer.length += uart_rx_buffer[0];
  	} else if(xbee_rx_buffer.pos < xbee_rx_buffer.length + 3){
  		xbee_rx_buffer.buffer[xbee_rx_buffer.pos-3] = uart_rx_buffer[0];
  		xbee_rx_buffer.checksum += uart_rx_buffer[0];
  	} else if(xbee_rx_buffer.pos >= xbee_rx_buffer.length + 3) {
  		xbee_rx_buffer.checksum = (uint8_t) xbee_rx_buffer.checksum;
  		xbee_rx_buffer.checksum = 0xFF - xbee_rx_buffer.checksum;
			if (osSemaphoreAcquire(ReceiveSemaphoreHandle, 0) == osOK) {
				if (xbee_rx_buffer.checksum == uart_rx_buffer[0]) {
					osEventFlagsSet(EventReceiveHandle,
							EVENT_RECEIVE_XBEE | EVENT_RECEIVE_SUCCESS);
				} else {
					osEventFlagsSet(EventReceiveHandle,
							EVENT_RECEIVE_XBEE | EVENT_RECEIVE_FAIL);
				}
				osSemaphoreRelease(ReceiveSemaphoreHandle);
			}
  		xbee_rx_buffer.isReceiving = FALSE;
  	}

  	++xbee_rx_buffer.pos;
  	HAL_UART_Receive_IT(&huart3, uart_rx_buffer, sizeof(uart_rx_buffer));
  }
}

static void _BMP390_Init(void){
	BMP390_AssignI2C(&hi2c1);
	BMP390_Init();
}
static void _BNO055_Init(void){
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();
}
static void _SD_Init(void){
	SD_Assign(&hspi2);
	retUSER = f_mount(&USERFatFS, USERPath, 1);
	if ( retUSER != FR_OK ) {
		loge("SD Init Fail");
	}
}
static void _Servo_Init(void){
	Servo_Attach(&hservo1, &htim3, TIM_CHANNEL_1);
	Servo_Attach(&hservo2, &htim3, TIM_CHANNEL_2);
	Servo_Attach(&hservo3, &htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_vMainTask */
/**
  * @brief  Function implementing the Main thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vMainTask */
void vMainTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  //HAL_UART_Receive_IT(&huart3, (uint8_t *) buffer, sizeof(buffer));
  //uint8_t data[]= {0x7E, 0x00, 0x0A, 0x01, 0x01, 0xCC, 0xCC, 0x00, 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x71};

  for(;;)
  {
    osDelay(1000);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    //HAL_UART_Transmit(&huart3, (uint8_t*)data, sizeof(data), 1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vGPSTask */
/**
* @brief Function implementing the GPS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vGPSTask */
void vGPSTask(void *argument)
{
  /* USER CODE BEGIN vGPSTask */
  /* Infinite loop */
	// Task for gathering NMEA message from GPS module using UART1 port
  for(;;)
  {
  	/*
  	if ( HAL_UART_Receive(&huart1, &IH_UART1_byteBuf, sizeof(IH_UART1_byteBuf), HAL_MAX_DELAY)!= HAL_OK){
  		continue;
  	}
  	if (IH_UART1_byteBuf == IH_UART1_HEADER1 && IH_UART1_headerPass == 0){
  		// the forward one is less frequent condition
  		IH_UART1_headerPass = 1;
  	}
  	else if (IH_UART1_headerPass == 1 && IH_UART1_byteBuf == IH_UART1_HEADER2){
  		IH_UART1_headerPass = 2;
  	}
  	else if (IH_UART1_headerPass == 2 && IH_UART1_byteBuf == IH_UART1_HEADER3){
  		IH_UART1_headerPass = 3;
  	}
  	else if (IH_UART1_headerPass == 3){
  		// message, start with comma(',')
  		if (IH_UART1_byteBuf != IH_UART1_TERMINATOR){
  			// not last byte
  			IH_UART1_buf[IH_UART1_pMessage] = IH_UART1_byteBuf;
  			IH_UART1_pMessage++;
  		}
  		else{
  			// last byte
  			IH_UART1_buf[IH_UART1_pMessage] = '\0';
  			IH_UART1_headerPass = 0;
  			IH_UART1_pMessage = 0;

  			strcpy(GPS_message, IH_UART1_buf);
  			// ToDo: request parsing
  			if (GPS_notReadFlag == 1){
  				GPS_overwriteFlag = 1;
  				GPS_onceOverwriteFlag = 1;
  			}
  			else{
  				GPS_notReadFlag = 1;
  			}
  		}
  	}
  	else{
  		// none of above
  		IH_UART1_headerPass = 0;
  		IH_UART1_pMessage = 0;
  	}
  	HAL_UART_Receive_IT(&huart1, &IH_UART1_byteBuf, sizeof(IH_UART1_byteBuf));
  	*/
  	osDelay(1);
  }
  /* USER CODE END vGPSTask */
}

/* USER CODE BEGIN Header_vStateManagingTask */
/**
* @brief Function implementing the StateManaging thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStateManagingTask */
void vStateManagingTask(void *argument)
{
  /* USER CODE BEGIN vStateManagingTask */
  /* Infinite loop */
  for(;;)
  {
  	osDelay(100);
  }
  /* USER CODE END vStateManagingTask */
}

/* USER CODE BEGIN Header_vReceiveTask */
/**
* @brief Function implementing the Receive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vReceiveTask */
void vReceiveTask(void *argument)
{
  /* USER CODE BEGIN vReceiveTask */
  /* Infinite loop */
	uint32_t event_flag;
	uint8_t message[RECEIVE_BUFFER_SIZE];
	uint16_t messageSize = 0;

	HAL_UART_Receive_IT(&huart3, uart_rx_buffer, sizeof(uart_rx_buffer));
  for(;;)
  {
  	event_flag = osEventFlagsWait(EventReceiveHandle, EVENT_RECEIVE_XBEE, osFlagsWaitAny, osWaitForever);

  	if(!(event_flag & EVENT_RECEIVE_XBEE)) { return; }
  	if((event_flag & EVENT_RECEIVE_SUCCESS)) {
  		if(xbee_rx_buffer.length < 5) { return; }
  		osSemaphoreAcquire(ReceiveSemaphoreHandle, osWaitForever);
  		memset(message, 0, sizeof(message));
  		memcpy(message, xbee_rx_buffer.buffer+5, xbee_rx_buffer.length-5);
  		messageSize = xbee_rx_buffer.length-5;
  		osSemaphoreRelease(ReceiveSemaphoreHandle);
  		logd("message:%s",message);
  	} else {
  		// fail
  	}
  	osDelay(1);
  }
  /* USER CODE END vReceiveTask */
}

/* USER CODE BEGIN Header_vDebugTask */
/**
* @brief Function implementing the Debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vDebugTask */
void vDebugTask(void *argument)
{
  /* USER CODE BEGIN vDebugTask */
  /* Infinite loop */
	uint32_t event_flag;
	uint32_t buffer;
	uint16_t cmd = 0;
	HAL_StatusTypeDef status;
	for (;;) {
		event_flag = osEventFlagsWait(EventReceiveHandle, EVENT_RECEIVE_USB, osFlagsWaitAny, osWaitForever);
		if (event_flag & EVENT_RECEIVE_USB) {
			Buzzer_Once();
			memcpy(&cmd, usb_rx_buffer.buffer, DEBUG_CMD_SIZE);
			switch (cmd) {
			case DEBUG_CMD_BUZZER:
				logd("Buzzer");
				osDelay(100);
				Buzzer_Once();
				osDelay(100);
				Buzzer_Once();
				break;
				/*
			case DEBUG_CMD_CALIBRATION:
				status = BMP390_ReadCalibration();
				if (status == HAL_OK) {
					logi("BMP390 CAL Complete.");
				} else {
					logi("BMP390 CAL Fail.");
				}
				break;
			case DEBUG_CMD_PRESSURE:
				logd("Pressure");

				if (BMP390_ReadRawPressure(&buffer, 10) != HAL_OK) {
					logi("BMP Read Pressure Error.");
				}
				sensor_data.pressure = BMP390_CompensatePressure(buffer);

				if (BMP390_ReadRawTemperature(&buffer, 10) != HAL_OK) {
					logi("BMP Read Temperature Error.");
				}
				sensor_data.temperature = BMP390_CompensateTemperature(buffer);

				logd("Temperature %f / pressure %f", sensor_data.temperature,
						sensor_data.pressure);

				break;
				*/
			}
		}
	}
  /* USER CODE END vDebugTask */
}

/* vSensorReadingCallback function */
void vSensorReadingCallback(void *argument)
{
  /* USER CODE BEGIN vSensorReadingCallback */

  /* USER CODE END vSensorReadingCallback */
}

/* vTransmitCallback function */
void vTransmitCallback(void *argument)
{
  /* USER CODE BEGIN vTransmitCallback */
	//logi("transmit");
  /* USER CODE END vTransmitCallback */
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
  printf("Error_Handler\r\n");
  __disable_irq();

  while (1)
  {
	  break;
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
