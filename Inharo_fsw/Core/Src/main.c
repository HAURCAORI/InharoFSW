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
	int altitude_updated_flag;
}SensorDataContainerTypeDef;

typedef enum{
	VEHICLE_RESET = 	0x00,
	F_LAUNCH_WAIT = 	0x01,
	F_ASCENT = 				0x02,
	F_HS_DEPLOYED = 	0x03,
	F_PC_DEPLOYED = 	0x04,
	F_LANDED = 				0x05,

	SIM_ENABLED = 		0x10,
	S_LAUNCH_WAIT = 	0x11,
	S_ASCENT = 				0x12,
	S_HS_DEPLOYED = 	0x13,
	S_PC_DEPLOYED = 	0x14,
	S_LANDED = 				0x15
}VehicleStateTypeDef;
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

// scale 100
#define VEHICLE_ASCENT_THRESHOLD 	500
#define VEHICLE_HS_THRESHOLD 			500
#define VEHICLE_PC_THRESHOLD 			10000
#define VEHICLE_LAND_THRESHOLD 		100

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

SensorDataContainerTypeDef sensor_data_container = {0, };
VehicleStateTypeDef vehicle_state = {0, };

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

// XBee 송신 관련 정의
uint8_t packet[PACKET_SIZE];
Telemetry telemetry;

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
  osTimerStart(SensorReadingHandle, 5000);
  osTimerStart(TransmitHandle, 5000);
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
	// only cares about automatic state transfer from altitude change
//	VehicleStateTypeDef vehicle_state;
	int32_t altitude = 0, old_altitude = 0, max_altitude = 0;
	int32_t s_altitude = 0, s_old_altitude = 0, s_max_altitude = 0;
  /* Infinite loop */
  for(;;)
  {
  	// altitude update
  	//while(sensor_data_container.altitude_updated_flag == 0); // false when the flag is set

  	// add blocking start here
  	old_altitude = altitude;
  	altitude = sensor_data_container.altitude;
  	sensor_data_container.altitude_updated_flag = 0;
  	if ( altitude > max_altitude ) max_altitude = altitude;

//  	s_old_altitude = altitude;
//  	s_altitude = sensor_data_container.s_altitude;
//  	sensor_data_container.s_altitude_updated_flag = 0;
//  	if ( s_altitude > s_max_altitude ) s_max_altitude = s_altitude;

  	switch(vehicle_state){
  	case F_LAUNCH_WAIT:
  		if (altitude > VEHICLE_ASCENT_THRESHOLD) {
  			vehicle_state = F_ASCENT;
  		}
  		break;
  	case F_ASCENT:
  		if (max_altitude - altitude > VEHICLE_HS_THRESHOLD){
  			// PC deploy if needed
  			vehicle_state = F_HS_DEPLOYED;
  		}
  		break;
  	case F_HS_DEPLOYED:
  		if (altitude < VEHICLE_PC_THRESHOLD){
  			// ToDo: implement PC deploy
  			vehicle_state = F_PC_DEPLOYED;
  		}
  		break;
  	case F_PC_DEPLOYED:
  		if ((old_altitude - altitude) < VEHICLE_LAND_THRESHOLD &&\
  				(old_altitude - altitude) > -VEHICLE_LAND_THRESHOLD ){
  			// ToDo: implement buzz
  			// implement CX OFF
  			vehicle_state = F_LANDED;
  		}
  		break;
  	case S_LAUNCH_WAIT:
  		if (s_altitude > VEHICLE_ASCENT_THRESHOLD) {
  			vehicle_state = S_ASCENT;
  		}
  		break;
  	case S_ASCENT:
  		if (s_max_altitude - s_altitude > VEHICLE_HS_THRESHOLD){
  			// PC deploy if needed
  			vehicle_state = S_HS_DEPLOYED;
  		}
  		break;
  	case S_HS_DEPLOYED:
  		if (s_altitude < VEHICLE_PC_THRESHOLD){
  			// ToDo: implement PC deploy
  			vehicle_state = S_PC_DEPLOYED;
  		}
  		break;
  	case S_PC_DEPLOYED:
  		if ((s_old_altitude - altitude) < VEHICLE_LAND_THRESHOLD &&\
  				(s_old_altitude - altitude) > -VEHICLE_LAND_THRESHOLD ){
  			// ToDo: implement buzz
  			// implement CX OFF
  			vehicle_state = S_LANDED;
  		}
  		break;
  	default:
  		// VEHICLE_RESET, F_LANDED, SIM_ENABLED, S_LANDED
  		// do nothing, basically...
  		// doing commanded task will be implemented in vRecieveTask(); function
  		break;
  	}
  	osDelay(100);
  	// add blocking end here
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
  sensor_data_container.altitude_updated_flag = 1;
  /* USER CODE END vSensorReadingCallback */
}

/* vTransmitCallback function */
void vTransmitCallback(void *argument)
{
  /* USER CODE BEGIN vTransmitCallback */
	//logi("transmit");

	telemetry.team_id = 1234;
	telemetry.hours = 12;
	telemetry.minutes = 34;
	telemetry.seconds = 56;
	telemetry.packet_count = 1;
	telemetry.mode = 0;
	telemetry.state = 1;
	telemetry.altitude = 100.0f;
	telemetry.air_speed = 4.2f;
	telemetry.heat_shield = 0;
	telemetry.parachute = 0;
	telemetry.temperature = 23.5f;
	telemetry.voltage = 3.3f;
	telemetry.pressure = 100.1f;
	telemetry.GPS_time_hours = 11;
	telemetry.GPS_time_minutes = 34;
	telemetry.GPS_time_seconds = 56;
	telemetry.GPS_time_subseconds = 234;
	telemetry.GPS_altitude = 100.3f;
	telemetry.GPS_latitude = 24.1f;
	telemetry.GPS_longitude = 26.2f;
	telemetry.GPS_sats = 7;
	telemetry.tilt_x = 32.0f;
	telemetry.tilt_y = 12.2f;
	telemetry.rot_z = 1.8f;
	telemetry.cmd_echo = 256;

	packet[0] = 0x7E;

	uint16_t length = TELEMETRY_PACKET_SIZE-4;
	packet[1] = length >> 8;
	packet[2] = (uint8_t) length;
	packet[3] = FRAME_TX;
	packet[4] = 0x01;
	packet[5] = FRAME_ADDRESS_HIGH;
	packet[6] = FRAME_ADDRESS_LOW;
	packet[7] = 0;
	memcpy(&packet[8], &telemetry, sizeof(telemetry));

	uint16_t checksum = 0;
	for(uint8_t i = 3; i < TELEMETRY_PACKET_SIZE-1; i++) {
		checksum += packet[i];
	}
	checksum = 0xFF - ((uint8_t) checksum);
	packet[TELEMETRY_PACKET_SIZE-1] = checksum;

	HAL_UART_Transmit(&huart3, &packet, sizeof(packet), 10);
	logd("Telemetry size:%d",sizeof(telemetry));
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
