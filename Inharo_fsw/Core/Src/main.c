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
#include "converter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	// bmp390, scale: 100
	float altitude;
	int64_t temperature;
	uint64_t pressure;
	// bno055
	double ang_x;
  double ang_y;
	double ang_z;
	double acc_x;
	double acc_y;
	double acc_z;
	double rot_x;
	double rot_y;
	double rot_z;
	double mag_x;
	double mag_y;
	double mag_z;
	// battery voltage, scale: 100(x)
	float battery_voltage;
	// airspeed, scale: 100(x)
	float air_speed;
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
  .priority = (osPriority_t) osPriorityNormal,
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
  .priority = (osPriority_t) osPriorityRealtime,
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
/* Definitions for ReceiveSemaphore */
osSemaphoreId_t ReceiveSemaphoreHandle;
const osSemaphoreAttr_t ReceiveSemaphore_attributes = {
  .name = "ReceiveSemaphore"
};
/* Definitions for CommandEvent */
osEventFlagsId_t CommandEventHandle;
const osEventFlagsAttr_t CommandEvent_attributes = {
  .name = "CommandEvent"
};
/* Definitions for EventReceive */
osEventFlagsId_t EventReceiveHandle;
const osEventFlagsAttr_t EventReceive_attributes = {
  .name = "EventReceive"
};
/* USER CODE BEGIN PV */
Servo_HandleTypeDef hservo1, hservo2, hservo3;
USB_Buffer_Type usb_rx_buffer;

SensorDataContainerTypeDef sensor_data_container = {0, };
VehicleStateTypeDef vehicle_state = {0, };

uint8_t IH_UART1_headerPass = 0;
uint8_t IH_UART1_pMessage = 0;
uint8_t IH_UART1_byteBuf = 0;
uint8_t IH_UART1_buf[IH_UART1_MAX_LENGTH] ={0,};


// Communication definition;
uint32_t packetCount = 0;
uint8_t uart1_rx_buffer[1];
uint8_t uart3_rx_buffer[1];
uint32_t receive_tick;
XBEE_Packet_Buffer xbee_rx_buffer;
GPS_Packet_Buffer gps_rx_buffer;

// XBee Sample
uint8_t tx_packet[PACKET_SIZE];
Telemetry telemetry;

// GPS
GPS_DataTypeDef gps_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void vMainTask(void *argument);
void vGPSTask(void *argument);
void vStateManagingTask(void *argument);
void vReceiveTask(void *argument);
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
  MX_DMA_Init();
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
  TransmitSemaphoreHandle = osSemaphoreNew(1, 1, &TransmitSemaphore_attributes);

  /* creation of ReceiveSemaphore */
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
  osTimerStart(SensorReadingHandle, 1000);
  osTimerStart(TransmitHandle, 1000);
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
  /* creation of CommandEvent */
  CommandEventHandle = osEventFlagsNew(&CommandEvent_attributes);

  /* creation of EventReceive */
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
	// USART1 - GPS Communication
	// USART3 - XBEE Communication
  if(huart->Instance == USART3)
  {
  	// Buffer Timeout
  	if(HAL_GetTick() - receive_tick > 100) {
  		xbee_rx_buffer.isReceiving = FALSE;
  		receive_tick = HAL_GetTick();
  	}

  	// Header Byte Detection
  	if(xbee_rx_buffer.isReceiving == FALSE) {
  		if(uart3_rx_buffer[0] == IH_UART3_HEADER){
  			memset(&xbee_rx_buffer,0, sizeof(xbee_rx_buffer));
  			xbee_rx_buffer.isReceiving = TRUE;
  	  	++xbee_rx_buffer.pos;
  		}
  		return;
  	}


  	// Get Data(0~1 : Length, 2~n : Data, n+1 : Checksum)
  	if(xbee_rx_buffer.pos == 0) {
  		xbee_rx_buffer.length = 0;
  	} else if(xbee_rx_buffer.pos == 1) {
  		xbee_rx_buffer.length = uart3_rx_buffer[0] * 0xFF;
  	} else if(xbee_rx_buffer.pos == 2) {
  		xbee_rx_buffer.length += uart3_rx_buffer[0];

  		// Invalid length data
  	  if(xbee_rx_buffer.length >= PACKET_SIZE) {
  	  	xbee_rx_buffer.isReceiving = FALSE;
  	  }
  	} else if(xbee_rx_buffer.pos < xbee_rx_buffer.length + IH_UART3_HEADER_OFFSET){
  		xbee_rx_buffer.buffer[xbee_rx_buffer.pos-IH_UART3_HEADER_OFFSET] = uart3_rx_buffer[0];
  		xbee_rx_buffer.checksum += uart3_rx_buffer[0];
  	} else if(xbee_rx_buffer.pos >= xbee_rx_buffer.length + IH_UART3_HEADER_OFFSET) {
  		// Checksum
  		xbee_rx_buffer.checksum = (uint8_t) xbee_rx_buffer.checksum;
			xbee_rx_buffer.checksum = 0xFF - xbee_rx_buffer.checksum;

			// Set Event Flags
			if(osSemaphoreAcquire(ReceiveSemaphoreHandle, 0) == osOK) {
				if (xbee_rx_buffer.checksum == uart3_rx_buffer[0]) {
					osEventFlagsSet(EventReceiveHandle, EVENT_RECEIVE_XBEE | EVENT_RECEIVE_SUCCESS);
				} else {
					osEventFlagsSet(EventReceiveHandle, EVENT_RECEIVE_XBEE | EVENT_RECEIVE_FAIL);
				}
				osSemaphoreRelease(ReceiveSemaphoreHandle);
			}
			xbee_rx_buffer.isReceiving = FALSE;
			return;
  	}

  	++xbee_rx_buffer.pos;

  	// Unlock Buffer
  	osSemaphoreRelease(ReceiveSemaphoreHandle);
  }
  else if(huart->Instance == USART1) {
		// Buffer Timeout
		if (HAL_GetTick() - receive_tick > 100) {
			gps_rx_buffer.isReceiving = FALSE;
			receive_tick = HAL_GetTick();
		}

		// Header Byte Detection
		if(gps_rx_buffer.isReceiving == FALSE) {
			if(uart1_rx_buffer[0] == IH_UART1_HEADER){
				memset(&gps_rx_buffer,0, sizeof(gps_rx_buffer));
				gps_rx_buffer.isReceiving = TRUE;
				++gps_rx_buffer.pos;
			}
			return;
		}

		// Get DeviceID
		if(gps_rx_buffer.pos == 1) {
			gps_rx_buffer.deviceID[0] = uart1_rx_buffer[0];
		} else if(gps_rx_buffer.pos == 2) {
			gps_rx_buffer.deviceID[1] = uart1_rx_buffer[0];
		}
		// Get SentenceID
		else if(gps_rx_buffer.pos == 3) {
			gps_rx_buffer.sentenceID[0] = uart1_rx_buffer[0];
		} else if(gps_rx_buffer.pos == 4) {
			gps_rx_buffer.sentenceID[1] = uart1_rx_buffer[0];
		} else if(gps_rx_buffer.pos == 5) {
			gps_rx_buffer.sentenceID[2] = uart1_rx_buffer[0];
		}
		// Get Checksum
		else if(gps_rx_buffer.pos == UINT16_MAX-1) {
			gps_rx_buffer.chk[0] = uart1_rx_buffer[0];
			++gps_rx_buffer.pos;
			return;
		} else if(gps_rx_buffer.pos == UINT16_MAX) {
			gps_rx_buffer.chk[1] = uart1_rx_buffer[0];

			// Set Event Flags
			if (osSemaphoreAcquire(ReceiveSemaphoreHandle, 0) == osOK) {
				if (gps_rx_buffer.checksum == HexCharToByte(gps_rx_buffer.chk[0], gps_rx_buffer.chk[1])) {
					osEventFlagsSet(EventReceiveHandle, EVENT_RECEIVE_GPS | EVENT_RECEIVE_SUCCESS);
				} else {
					osEventFlagsSet(EventReceiveHandle, EVENT_RECEIVE_GPS | EVENT_RECEIVE_FAIL);
				}
				osSemaphoreRelease(ReceiveSemaphoreHandle);
			}

			gps_rx_buffer.isReceiving = FALSE;
			return;
		}
		// Get Data
		else if(gps_rx_buffer.pos > 5) {
			// Invalid length
			if(gps_rx_buffer.pos - IH_UART1_HEADER_OFFSET >= PACKET_SIZE) {
				gps_rx_buffer.isReceiving = FALSE;
				return;
			}

			// Termination
			if(uart1_rx_buffer[0] == IH_UART1_TERMINATOR) {
				gps_rx_buffer.pos = UINT16_MAX-1;
				return;
			}

			// Data Write
			gps_rx_buffer.buffer[gps_rx_buffer.pos - IH_UART1_HEADER_OFFSET] = uart1_rx_buffer[0];
			++gps_rx_buffer.length;
		}
		//Checksum and Increment
		gps_rx_buffer.checksum ^= uart1_rx_buffer[0];
		++gps_rx_buffer.pos;

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
	uint32_t flags;
	GPS_Packet packet;

	uint32_t sentence;

	HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, sizeof(uart1_rx_buffer));
  for(;;)
  {
  	flags = osEventFlagsWait(EventReceiveHandle, EVENT_RECEIVE_GPS, osFlagsWaitAny, osWaitForever);

  	// Validation check
  	if(!(flags & EVENT_RECEIVE_GPS)){ continue; }
  	if(!(flags & EVENT_RECEIVE_SUCCESS)) { continue; }
		osSemaphoreAcquire(ReceiveSemaphoreHandle, 10);
		memset(&packet, 0, sizeof(packet));
		memcpy(packet.data, gps_rx_buffer.buffer, gps_rx_buffer.length);
		memcpy(packet.deviceID, gps_rx_buffer.deviceID,2);
		memcpy(packet.sentenceID, gps_rx_buffer.sentenceID,3);
		packet.length = gps_rx_buffer.length;
		osSemaphoreRelease(ReceiveSemaphoreHandle);

		sentence = (packet.sentenceID[0] << 16) | (packet.sentenceID[1] << 8) | (packet.sentenceID[2]);
		switch(sentence) {
		case GPS_SENTENCE_GGA: {
			GPS_NMEA_parseGGA(packet.data, &gps_data);
			//logd("GPS/%s/%s",packet.deviceID, packet.data);
		}
		}
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
	uint32_t flags;
	XBEE_Packet packet;
	uint8_t RFdata[PACKET_SIZE];

	HAL_UART_Receive_DMA(&huart3, uart3_rx_buffer, sizeof(uart3_rx_buffer));
  for(;;)
  {
  	flags = osEventFlagsWait(EventReceiveHandle, EVENT_RECEIVE_XBEE, osFlagsWaitAny, osWaitForever);

  	// Validation Check;
  	if(!(flags & EVENT_RECEIVE_XBEE)) { continue; }
  	if(!(flags & EVENT_RECEIVE_SUCCESS)) { continue; }

		// Block to acquire data
		osSemaphoreAcquire(ReceiveSemaphoreHandle, 10);
		memset(&packet, 0, sizeof(packet));
		memcpy(packet.data, xbee_rx_buffer.buffer, xbee_rx_buffer.length);
		packet.length = xbee_rx_buffer.length;
		osSemaphoreRelease(ReceiveSemaphoreHandle);

		// Packet check
		if (packet.length < 6) { continue; }

		// Processing
  	switch(packet.data[0]) {
  	case FRAME_TYPE_RX: {
  		memset(RFdata, 0, sizeof(RFdata));
  		memcpy(RFdata, packet.data+5, packet.length-5);
  		logd("RFdata:%s", RFdata);
  	}
  	}
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
	double ang_x, ang_y, ang_z;
	int16_t ADC1_CH0, ADC1_CH1;
	float altitude;
	float battery_voltage;
	float air_speed;

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
	bno055vector = bno055_getVectorEuler();
  ang_x = bno055vector.x;
  ang_y = bno055vector.y;
  ang_z = bno055vector.z;


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
  altitude = (powf(pressure_ratio, ALTITUDE_POWER_COEFFICIENT) - 1) * ALTITUDE_PRODUCT_COEFFICIENT; // *100

	// calculate battery voltage
  //battery_voltage = ADC1_CH0 / 4015 * 3.3 * 1.5 * 100;
  //battery_voltage = ADC1_CH0 * 0.123287671232877;
  battery_voltage = ADC1_CH0 / 4095.0 * 3.3 * 1.5;

	// calculate air speed
  air_speed = DP_calculateAirSpeedComp(ADC1_CH1, pressure / 100.f, temperature / 100.f);
  air_speed /= 100;

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
  sensor_data_container.ang_x = ang_x;
  sensor_data_container.ang_y = ang_y;
  sensor_data_container.ang_z = ang_z;
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
	UpdateTime();
	telemetry.team_id = 1234;
	telemetry.hours = sTime.Hours;
	telemetry.minutes = sTime.Minutes;
	telemetry.seconds = sTime.Seconds;
	telemetry.subseconds = g_SubSeconds;
	telemetry.packet_count = packetCount++;
	telemetry.mode = 0;
	telemetry.state = 1;
	telemetry.altitude = sensor_data_container.altitude;
	telemetry.air_speed = sensor_data_container.air_speed;
	telemetry.heat_shield = 0;
	telemetry.parachute = 0;
	telemetry.temperature = sensor_data_container.temperature;
	telemetry.voltage = sensor_data_container.battery_voltage;
	telemetry.pressure = sensor_data_container.pressure;
	telemetry.GPS_time_hours = gps_data.hours;
	telemetry.GPS_time_minutes = gps_data.minutes;
	telemetry.GPS_time_seconds = gps_data.seconds;
	telemetry.GPS_altitude = gps_data.altitude;
	telemetry.GPS_latitude = gps_data.latitude;
	telemetry.GPS_longitude = gps_data.longitude;
	telemetry.GPS_sats = gps_data.satellites;
	telemetry.tilt_x = sensor_data_container.ang_z; // Axis Change
	telemetry.tilt_y = sensor_data_container.ang_y;
	telemetry.rot_z = sensor_data_container.rot_z;
	telemetry.cmd_echo = 0xFF;

	//packet header
	tx_packet[0] = 0x7E;

	uint16_t length = TELEMETRY_PACKET_SIZE-4;
	tx_packet[1] = length >> 8;
	tx_packet[2] = (uint8_t) length;
	tx_packet[3] = FRAME_TYPE_TX;
	tx_packet[4] = 0x01;
	tx_packet[5] = FRAME_ADDRESS_HIGH;
	tx_packet[6] = FRAME_ADDRESS_LOW;
	tx_packet[7] = 0;
	memcpy(&tx_packet[8], &telemetry, sizeof(telemetry));

	uint16_t checksum = 0;
	for(uint8_t i = 3; i < TELEMETRY_PACKET_SIZE-1; i++) {
		checksum += tx_packet[i];
	}
	checksum = 0xFF - ((uint8_t) checksum);
	tx_packet[TELEMETRY_PACKET_SIZE-1] = checksum;
	HAL_UART_Transmit(&huart3, tx_packet, sizeof(tx_packet), 0);
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
