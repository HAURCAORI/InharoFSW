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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	// bmp390, scale: 100
	int32_t altitude;
	int64_t temperature;
	uint64_t pressure;
	// bno055
	double acc_x;
	double acc_y;
	double acc_z;
	double rot_x;
	double rot_y;
	double rot_z;
	double mag_x;
	double mag_y;
	double mag_z;
	// battery voltage, scale: 100
	uint16_t battery_voltage;
	// airspeed, scale: 100
	uint16_t air_speed;
}SensorDataContainerTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALTITUDE_POWER_COEFFICIENT ( (double) 0.190263105239812 )
#define ALTITUDE_PRODUCT_COEFFICIENT ( (double) -4.433076923076923e+4)

#define IH_UART1_MAX_LENGTH (80)
#define IH_UART1_HEADER1 ('G')
#define IH_UART1_HEADER2 ('G')
#define IH_UART1_HEADER3 ('A')
#define IH_UART1_TERMINATOR ('*')

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
osSemaphoreId_t TransmitSemaphoreHandle;
const osSemaphoreAttr_t TransmitSemaphore_attributes = {
  .name = "TransmitSemaphore"
};
/* Definitions for USBEvent */
osEventFlagsId_t USBEventHandle;
const osEventFlagsAttr_t USBEvent_attributes = {
  .name = "USBEvent"
};
/* Definitions for GPSEvent */
osEventFlagsId_t GPSEventHandle;
const osEventFlagsAttr_t GPSEvent_attributes = {
  .name = "GPSEvent"
};
/* Definitions for CommandEvent */
osEventFlagsId_t CommandEventHandle;
const osEventFlagsAttr_t CommandEvent_attributes = {
  .name = "CommandEvent"
};
/* USER CODE BEGIN PV */

Servo_HandleTypeDef hservo1, hservo2, hservo3;
USB_Buffer_Type usb_rx_buffer;
volatile Sensor_Data sensor_data;

SensorDataContainerTypeDef sensor_data_container = {0,};

uint8_t IH_UART1_headerPass = 0;
uint8_t IH_UART1_pMessage = 0;
uint8_t IH_UART1_byteBuf = 0;
uint8_t IH_UART1_buf[IH_UART1_MAX_LENGTH] ={0,};

uint8_t GPS_message[GPS_MAX_LENGTH] = {0,};
uint8_t GPS_notReadFlag = 1;
uint8_t GPS_overwriteFlag = 1;
uint8_t GPS_onceOverwriteFlag = 1;

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
  _BMP390_Init();
  _BNO055_Init();
  _SD_Init();
  _Servo_Init();


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of TransmitSemaphore */
  TransmitSemaphoreHandle = osSemaphoreNew(1, 1, &TransmitSemaphore_attributes);

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
  /* creation of USBEvent */
  USBEventHandle = osEventFlagsNew(&USBEvent_attributes);

  /* creation of GPSEvent */
  GPSEventHandle = osEventFlagsNew(&GPSEvent_attributes);

  /* creation of CommandEvent */
  CommandEventHandle = osEventFlagsNew(&CommandEvent_attributes);

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
	  //HAL_UART_Receive_IT(&huart3, (uint8_t *)buffer, sizeof(buffer));
    //logi("buffer");
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
	if ( retUSER != FR_OK ) Error_Handler();
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

  //HAL_UART_Receive_IT(&huart3, (uint8_t *) buffer, sizeof(buffer));
  //uint8_t data[]= {0x7E, 0x00, 0x0A, 0x01, 0x01, 0xCC, 0xCC, 0x00, 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x71};
  logi("Initializing...");
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  osDelay(100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  logi("Initialized");
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
    osDelay(1);
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
  for(;;)
  {
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
		event_flag = osEventFlagsWait(USBEventHandle, RECEIVED_USB, osFlagsWaitAny, 1000);
		if (event_flag & RECEIVED_USB) {
			Buzzer_Once();
//			memcpy(&cmd, usb_rx_buffer.buffer, DEBUG_CMD_SIZE);
//			switch (cmd) {
//			case DEBUG_CMD_BUZZER:
//				logd("Buzzer");
//				osDelay(100);
//				Buzzer_Once();
//				osDelay(100);
//				Buzzer_Once();
//				break;
//			case DEBUG_CMD_CALIBRATION:
//				status = BMP390_ReadCalibration();
//				if (status == HAL_OK) {
//					logi("BMP390 CAL Complete.");
//				} else {
//					logi("BMP390 CAL Fail.");
//				}
//				break;
//			case DEBUG_CMD_PRESSURE:
//				logd("Pressure");
//
//				if (BMP390_ReadRawPressure(&buffer, 10) != HAL_OK) {
//					logi("BMP Read Pressure Error.");
//				}
//				sensor_data.pressure = BMP390_CompensatePressure(buffer);
//
//				if (BMP390_ReadRawTemperature(&buffer, 10) != HAL_OK) {
//					logi("BMP Read Temperature Error.");
//				}
//				sensor_data.temperature = BMP390_CompensateTemperature(buffer);
//
//				logd("Temperature %f / pressure %f", sensor_data.temperature, sensor_data.pressure);
//
//				break;
//			}
		}
	}
  /* USER CODE END vDebugTask */
}

/* vSensorReadingCallback function */
void vSensorReadingCallback(void *argument)
{
  /* USER CODE BEGIN vSensorReadingCallback */
	int64_t temperature;
	uint64_t pressure;
	bno055_vector_t bno055vector;
	double acc_x, acc_y, acc_z;
	double rot_x, rot_y, rot_z;
	double mag_x, mag_y, mag_z;
	int16_t ADC1_CH0, ADC1_CH1;
	int32_t altitude;
	uint16_t battery_voltage;
	uint16_t air_speed;

	// read bmp390
	BMP390_GetValue(&temperature, &pressure, 50);

	// read bno055
	bno055vector = bno055_getVectorAccelerometer();
	acc_x = bno055vector.x;
	acc_y = bno055vector.y;
	acc_z = bno055vector.z;
	bno055vector = bno055_getVectorGyroscope();
	rot_x = bno055vector.x;
	rot_y = bno055vector.y;
	rot_z = bno055vector.z;
	bno055vector = bno055_getVectorMagnetometer();
	mag_x = bno055vector.x;
	mag_y = bno055vector.y;
	mag_z = bno055vector.z;

	// read ADC1 CH0
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 50);
	ADC1_CH0 = HAL_ADC_GetValue(&hadc1);

	// read ADC1 CH1
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 50);
	ADC1_CH1 = HAL_ADC_GetValue(&hadc1);

	// calculate altitude
  /*
  	 * Solve:
  	 *
  	 *     /        -(aR / g)     \
  	 *    | (p / p1)          . T1 | - T1
  	 *     \                      /
  	 * h = -------------------------------  + h1
  	 *                   a
  	 * a:  temperature gradient
  	 * R:  gas constant of air
  	 * g:  gravity constant
  	 * p:  pressure
  	 * p1: pressure, sea level
  	 * T1: temperature, sea level
  	 */
  //ToDo: get sea level pressure (calibrated)from RTC backup register
//  double pressure_sea_level = 101325*100;
//  double pressure_ratio = pressure / pressure_sea_level;
  double pressure_ratio = pressure * 9.869232667160128e-4;
  altitude = (powf(pressure_ratio, ALTITUDE_POWER_COEFFICIENT) - 1) * ALTITUDE_PRODUCT_COEFFICIENT * 100;

	// calculate battery voltage
  //battery_voltage = ADC1_CH0 / 4015 * 3.3 * 1.5 * 100;
  battery_voltage = ADC1_CH0 * 0.123287671232877;

	// calculate air speed
  air_speed = DP_calculateAirSpeedComp(ADC1_CH1, pressure / 100.f, temperature / 100.f);

	// move data to sensor data container
  // ToDo: block other task and move data
  sensor_data_container.pressure = pressure;
  sensor_data_container.temperature = temperature;
  sensor_data_container.acc_x = acc_x;
  sensor_data_container.acc_y = acc_y;
  sensor_data_container.acc_z = acc_z;
  sensor_data_container.rot_x = rot_x;
  sensor_data_container.rot_y = rot_y;
  sensor_data_container.rot_z = rot_z;
  sensor_data_container.mag_x = mag_x;
  sensor_data_container.mag_y = mag_y;
  sensor_data_container.mag_z = mag_z;
  sensor_data_container.altitude = altitude;
  sensor_data_container.battery_voltage = battery_voltage;
  sensor_data_container.air_speed = air_speed;

  /* USER CODE END vSensorReadingCallback */
}

/* vTransmitCallback function */
void vTransmitCallback(void *argument)
{
  /* USER CODE BEGIN vTransmitCallback */

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
