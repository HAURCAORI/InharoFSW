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
#include "telemetryHandler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	// bmp390, scale: 100
	float altitude;
	float s_altitude;
	int64_t temperature;
	uint64_t pressure;
	uint64_t s_pressure;
	// bno055
	double tilt_x;
	double tilt_y;
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
	// adc
	uint16_t adc_0;
	uint16_t adc_1;
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

typedef enum{
	CMD_ERR,
	CMD_ERR_ARG,
	CMD_ERR_COM,

	CMD_CX_ON,
	CMD_CX_OFF,

	CMD_ST_TIME,
	CMD_ST_GPS,

	CMD_SIM_ENABLE,
	CMD_SIM_ACTIVATE,
	CMD_SIM_DISABLE,

	CMD_SIMP,

	CMD_CAL,

	CMD_BCN_ON,
	CMD_BCN_OFF,

	CMD_DEP_HS,
	CMD_DEP_PC,

	CMD_REL_HS,

	CMD_TEST,

	CMD_INIT,

	CMD_RESET
}CMD_CommandCaseTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALTITUDE_POWER_COEFFICIENT ( (double) 0.190263105239812 )
#define ALTITUDE_PRODUCT_COEFFICIENT ( (double) -4.433076923076923e+4)
double r_pressure_sea_level = 1 / ( 101325* 100 );


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
VehicleStateTypeDef vehicle_state = VEHICLE_RESET;

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
int Calibrate(void);
void Backup(void);
void BackupRecovery(void);
CMD_CommandCaseTypeDef CMD_parseCommandMessage(uint8_t *message, int *pArg);
CMD_CommandCaseTypeDef CMD_parseTime(uint8_t *message, int *pArg);
CMD_CommandCaseTypeDef CMD_parseSIMP(uint8_t *message, int *pArg);
void CMD_excuteCX_ON(void);
void CMD_excuteCX_OFF(void);
void CMD_excuteST_TIME(int argument);
void CMD_excuteST_GPS(void);
void CMD_excuteSIM_ENABLE(void);
void CMD_excuteSIM_ACTIVATE(void);
void CMD_excuteSIM_DISABLE(void);
void CMD_excuteSIMP(int argument);
void CMD_excuteCAL(void);
void CMD_excuteBCN_ON(void);
void CMD_excuteBCN_OFF(void);
void CMD_excuteDEP_HS(void);
void CMD_excuteDEP_PC(void);
void CMD_excuteREL_HS(void);
void CMD_excuteINIT(void);
void CMD_excuteRESET(void);

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
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0){
  	BackupRecovery();
  }

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
  osTimerStart(TransmitHandle, 100);
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
	Servo_Attach(&hservo1, &htim3, TIM_CHANNEL_1);	// heat shield deploy servo
	Servo_Attach(&hservo2, &htim3, TIM_CHANNEL_2);	// heat shield release servo
	Servo_Attach(&hservo3, &htim3, TIM_CHANNEL_3);	// parachute deploy servo
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
	Servo_Write(&hservo1, 0);
	Servo_Write(&hservo2, 0);
	Servo_Write(&hservo3, 0);
}

int Calibrate(void){
	bno055_calibration_data_t bno055_cal_data;

	int32_t cal_zero_altitude0 = 0;
	int32_t cal_zero_altitude1 = 0;
	int32_t	cal_zero_velocity0 = 0;
	int32_t	cal_zero_velocity1 = 0;
	int32_t	cal_gyro_ofst_x_gyro_ofst_y = 0;
	int32_t	cal_gyro_ofst_z_mag_ofst_x = 0;
	int32_t	cal_mag_ofst_y_mag_ofst_z = 0;
	int32_t	cal_acc_ofst_x_acc_ofst_y = 0;
	int32_t	cal_acc_ofst_z_reserved = 0;
	int32_t	cal_imu_radius_mag_acc = 0;

	// get bno055 calibration data
	if(bno055_getCalibrationState().sys < 0x03) return 1;
//	if(bno055_getCalibrationState() < 0x02) return 1;
//	if(bno055_getCalibrationState() < 0x01) return 1;
	bno055_cal_data = bno055_getCalibrationData();

	// zero altitude calibration; simulation pressure is calibrated when first SIMP command received
	r_pressure_sea_level = 1 / sensor_data_container.pressure;

	// calibrate velocity calculation
	DP_calcCalibrationFromADC(sensor_data_container.adc_1);

	// make calibration data for RTC backup register
	union{
		double  val_double;
		int32_t val_int32[2];
	}double_and_int32;

	double_and_int32.val_double = r_pressure_sea_level;
	cal_zero_altitude0 = double_and_int32.val_int32[0];
	cal_zero_altitude1 = double_and_int32.val_int32[1];

	double_and_int32.val_double = DP_getCalibration();
	cal_zero_velocity0 = double_and_int32.val_int32[0];
	cal_zero_velocity1 = double_and_int32.val_int32[1];

	cal_gyro_ofst_x_gyro_ofst_y = (bno055_cal_data.offset.gyro.x 	<< 16)|(bno055_cal_data.offset.gyro.y);
	cal_gyro_ofst_z_mag_ofst_x  = (bno055_cal_data.offset.gyro.z 	<< 16)|(bno055_cal_data.offset.mag.x);
	cal_mag_ofst_y_mag_ofst_z   = (bno055_cal_data.offset.mag.y  	<< 16)|(bno055_cal_data.offset.mag.z);
	cal_acc_ofst_x_acc_ofst_y		= (bno055_cal_data.offset.accel.x	<< 16)|(bno055_cal_data.offset.accel.y);
	cal_acc_ofst_z_reserved			= (bno055_cal_data.offset.accel.z	<< 16);
	cal_imu_radius_mag_acc			= (bno055_cal_data.radius.mag			<< 16)|(bno055_cal_data.radius.accel);

	// save calibration data
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2,		cal_zero_altitude0);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3,		cal_zero_altitude1);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR6,		cal_zero_velocity0);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7,		cal_zero_velocity1);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR8,  	cal_gyro_ofst_x_gyro_ofst_y);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR9,  	cal_gyro_ofst_z_mag_ofst_x);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR10, 	cal_mag_ofst_y_mag_ofst_z);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR11, 	cal_acc_ofst_x_acc_ofst_y);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR12, 	cal_acc_ofst_z_reserved);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR13, 	cal_imu_radius_mag_acc);
  HAL_PWR_DisableBkUpAccess();

	return 0;
}
void Backup(void){
	bno055_calibration_data_t bno055_cal_data;
	uint32_t	bkp_vehicle =0;
	uint32_t	bkp_packet_count = 0;
	uint32_t 	cal_zero_altitude0 = 0;
	uint32_t 	cal_zero_altitude1 = 0;
	uint32_t 	cal_s_zero_altitude0 = 0;
	uint32_t 	cal_s_zero_altitude1 = 0;
	uint32_t	cal_zero_velocity0 = 0;
	uint32_t	cal_zero_velocity1 = 0;
	uint32_t	cal_gyro_ofst_x_gyro_ofst_y = 0;
	uint32_t	cal_gyro_ofst_z_mag_ofst_x = 0;
	uint32_t	cal_mag_ofst_y_mag_ofst_z = 0;
	uint32_t	cal_acc_ofst_x_acc_ofst_y = 0;
	uint32_t	cal_acc_ofst_z_reserved = 0;
	uint32_t	cal_rad_mag_rad_acc = 0;

	union{
		uint8_t 	val_uint8[4];
		uint32_t	val_uint32;
	}uint8_to_uint32;
	union{
		double 		val_double;
		uint32_t 	val_uint32[2];
	}double_to_uint32;
	union{
		int16_t 	val_int16[2];
		uint32_t 	val_uint32;
	}int16_to_uint32;
	union{
		int16_t 	val_uint16[2];
		uint32_t 	val_uint32;
	}uint16_to_uint32;

	uint8_to_uint32.val_uint8[0] 		= 0;
	uint8_to_uint32.val_uint8[1] 		= 0;
	uint8_to_uint32.val_uint8[2] 		= ( uint8_t ) vehicle_state;
//	uint8_to_uint32.val_uint8[3]	 	= ( uint8_t ) ( ( ( val_CX & 0x01 ) << 1 ) | 0x01 );
	uint8_to_uint32.val_uint8[3] 		= 1;
	bkp_vehicle 										= uint8_to_uint32.val_uint32;

	bkp_packet_count 								= packetCount;

	double_to_uint32.val_double 		= r_pressure_sea_level;
	cal_zero_altitude0 							= double_to_uint32.val_uint32[0];
	cal_zero_altitude1 							= double_to_uint32.val_uint32[1];

//	double_to_uint32.val_double 	= r_s_pressure_sea_level;
//	cal_s_zero_altitude0 					= double_to_uint32.val_uint32[0];
//	cal_s_zero_altitude1 					= double_to_uint32.val_uint32[1];

	double_to_uint32.val_double			= DP_getCalibration();
	cal_zero_velocity0 							= double_to_uint32.val_uint32[0];
	cal_zero_velocity1 							= double_to_uint32.val_uint32[1];

	bno055_cal_data = bno055_getCalibrationData();

	int16_to_uint32.val_int16[0] 		= bno055_cal_data.offset.gyro.x;
	int16_to_uint32.val_int16[1] 		= bno055_cal_data.offset.gyro.y;
	cal_gyro_ofst_x_gyro_ofst_y 		= int16_to_uint32.val_uint32;

	int16_to_uint32.val_int16[0] 		= bno055_cal_data.offset.gyro.z;
	int16_to_uint32.val_int16[1] 		= bno055_cal_data.offset.mag.x;
	cal_gyro_ofst_z_mag_ofst_x  		=	int16_to_uint32.val_uint32;

	int16_to_uint32.val_int16[0] 		= bno055_cal_data.offset.mag.y;
	int16_to_uint32.val_int16[1] 		= bno055_cal_data.offset.mag.z;
	cal_mag_ofst_y_mag_ofst_z   		= int16_to_uint32.val_uint32;

	int16_to_uint32.val_int16[0] 		= bno055_cal_data.offset.accel.x;
	int16_to_uint32.val_int16[1] 		= bno055_cal_data.offset.accel.y;
	cal_acc_ofst_x_acc_ofst_y   		= int16_to_uint32.val_uint32;

	int16_to_uint32.val_int16[0] 		= bno055_cal_data.offset.accel.z;
	int16_to_uint32.val_int16[1] 		= 0;
	cal_acc_ofst_z_reserved     		= int16_to_uint32.val_uint32;

	uint16_to_uint32.val_uint16[0] 	= bno055_cal_data.radius.mag;
	uint16_to_uint32.val_uint16[1] 	= bno055_cal_data.radius.accel;
	cal_rad_mag_rad_acc     				= uint16_to_uint32.val_uint32;

	// save calibration data
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0,		bkp_vehicle);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1,		bkp_packet_count);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2,		cal_zero_altitude0);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3,		cal_zero_altitude1);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4,		cal_s_zero_altitude0);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5,		cal_s_zero_altitude1);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR6,		cal_zero_velocity0);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7,		cal_zero_velocity1);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR8,  	cal_gyro_ofst_x_gyro_ofst_y);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR9,  	cal_gyro_ofst_z_mag_ofst_x);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR10, 	cal_mag_ofst_y_mag_ofst_z);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR11, 	cal_acc_ofst_x_acc_ofst_y);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR12, 	cal_acc_ofst_z_reserved);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR13, 	cal_rad_mag_rad_acc);
  HAL_PWR_DisableBkUpAccess();

	return 0;

}
void BackupRecovery(void){
	bno055_calibration_data_t bno055_cal_data;
	uint32_t 	bkp_vehicle;
	uint32_t 	bkp_packet_count;
	uint32_t 	cal_zero_altitude0;
	uint32_t 	cal_zero_altitude1;
	uint32_t 	cal_s_zero_altitude0;
	uint32_t 	cal_s_zero_altitude1;
	uint32_t	cal_zero_velocity0;
	uint32_t	cal_zero_velocity1;
	uint32_t	cal_gyro_ofst_x_gyro_ofst_y;
	uint32_t	cal_gyro_ofst_z_mag_ofst_x;
	uint32_t	cal_mag_ofst_y_mag_ofst_z;
	uint32_t	cal_acc_ofst_x_acc_ofst_y;
	uint32_t	cal_acc_ofst_z_reserved;
	uint32_t	cal_rad_mag_rad_acc;

	// load calibration data / backup data
	HAL_PWR_EnableBkUpAccess();
	bkp_vehicle 								= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
	bkp_packet_count 						= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	cal_zero_altitude0 					= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
	cal_zero_altitude1 					= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
	cal_s_zero_altitude0 				= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);
	cal_s_zero_altitude1 				= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5);
	cal_zero_velocity0 					= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR6);
	cal_zero_velocity1 					= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7);
	cal_gyro_ofst_x_gyro_ofst_y	= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR8);
	cal_gyro_ofst_z_mag_ofst_x 	= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR9);
	cal_mag_ofst_y_mag_ofst_z 	= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR10);
	cal_acc_ofst_x_acc_ofst_y 	= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR11);
	cal_acc_ofst_z_reserved 		= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR12);
	cal_rad_mag_rad_acc 				= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR13);
	HAL_PWR_DisableBkUpAccess();

	union{
		uint32_t	val_uint32;
		uint8_t 	val_uint8[4];
	}uint32_to_uint8;
	union{
		uint32_t 	val_uint32[2];
		double 		val_double;
	}uint32_to_double;
	union{
		uint32_t 	val_uint32;
		int16_t 	val_int16[2];
	}uint32_to_int16;
	union{
		uint32_t 	val_uint32;
		int16_t 	val_uint16[2];
	}uint32_to_uint16;


	uint32_to_uint8.val_uint32 = bkp_vehicle;
	vehicle_state = uint32_to_uint8.val_uint8[2];
//	val_CX = 0b00000010U & uint32_to_uint8.val_uint8[3];

	packetCount = bkp_packet_count;

	uint32_to_double.val_uint32[0] = cal_zero_altitude0;
	uint32_to_double.val_uint32[1] = cal_zero_altitude1;
	r_pressure_sea_level = uint32_to_double.val_double;

	uint32_to_double.val_uint32[0] = cal_s_zero_altitude0;
	uint32_to_double.val_uint32[1] = cal_s_zero_altitude1;
//	r_s_pressure_sea_level = uint32_to_double.val_double;

	uint32_to_double.val_uint32[0] = cal_zero_velocity0;
	uint32_to_double.val_uint32[1] = cal_zero_velocity1;
	DP_setCalibrationFromDouble(uint32_to_double.val_double);

	uint32_to_int16.val_uint32 = cal_gyro_ofst_x_gyro_ofst_y;
	bno055_cal_data.offset.gyro.x = uint32_to_int16.val_int16[0];
	bno055_cal_data.offset.gyro.y = uint32_to_int16.val_int16[1];

	uint32_to_int16.val_uint32 = cal_gyro_ofst_z_mag_ofst_x;
	bno055_cal_data.offset.gyro.z = uint32_to_int16.val_int16[0];
	bno055_cal_data.offset.mag.x = uint32_to_int16.val_int16[1];

	uint32_to_int16.val_uint32 = cal_mag_ofst_y_mag_ofst_z;
	bno055_cal_data.offset.mag.y = uint32_to_int16.val_int16[0];
	bno055_cal_data.offset.mag.z = uint32_to_int16.val_int16[1];

	uint32_to_int16.val_uint32 = cal_acc_ofst_x_acc_ofst_y;
	bno055_cal_data.offset.accel.x = uint32_to_int16.val_int16[0];
	bno055_cal_data.offset.accel.y = uint32_to_int16.val_int16[1];

	uint32_to_int16.val_uint32 = cal_acc_ofst_z_reserved;
	bno055_cal_data.offset.accel.z = uint32_to_int16.val_int16[0];

	uint32_to_uint16.val_uint32 = cal_rad_mag_rad_acc;
	bno055_cal_data.radius.mag = uint32_to_uint16.val_uint16[0];
	bno055_cal_data.radius.accel = uint32_to_uint16.val_uint16[1];

	bno055_setCalibrationData(bno055_cal_data);

	return;
}
void CMD_commandHandler(uint8_t *message){
	int argument;
	CMD_CommandCaseTypeDef ret;

	ret = CMD_parseCommandMessage(message, &argument);
	switch(ret){
	case CMD_ERR:
		loge("unknown command parser error, possibly a null pointer error. \n");
		break;
	case CMD_ERR_ARG:
		loge("wrong command argument error.\n");
		break;
	case CMD_ERR_COM:
		loge("wrong command error.\n");
		break;
	case CMD_CX_ON:
		CMD_excuteCX_ON();
		break;
	case CMD_CX_OFF:
		CMD_excuteCX_OFF();
		break;
	case CMD_ST_TIME:
		CMD_excuteST_TIME(argument);
		break;
	case CMD_ST_GPS:
		CMD_excuteST_GPS();
		break;
	case CMD_SIM_ENABLE:
		CMD_excuteSIM_ENABLE();
		break;
	case CMD_SIM_ACTIVATE:
		CMD_excuteSIM_ACTIVATE();
		break;
	case CMD_SIM_DISABLE:
		CMD_excuteSIM_DISABLE();
		break;
	case CMD_SIMP:
		CMD_excuteSIMP(argument);
		break;
	case CMD_CAL:
		CMD_excuteCAL();
		break;
	case CMD_BCN_ON:
		CMD_excuteBCN_ON();
		break;
	case CMD_BCN_OFF:
		CMD_excuteBCN_OFF();
		break;
	case CMD_DEP_HS:
		CMD_excuteDEP_HS();
		break;
	case CMD_DEP_PC:
		CMD_excuteDEP_PC();
		break;
	case CMD_REL_HS:
		CMD_excuteREL_HS();
		break;
	case CMD_TEST:
		logd("CMD_TEST not implemented yet.\n");
		break;
	case CMD_INIT:
		CMD_excuteINIT();
		break;
	case CMD_RESET:
		CMD_excuteRESET();
		break;
	default:
		// never occurs
		Error_Handler();
		break;
	}
}
CMD_CommandCaseTypeDef CMD_parseCommandMessage(uint8_t *message, int *pArg){
	uint8_t *temp;


	if (message == NULL || pArg == NULL){
		return CMD_ERR;
	}
	if (strncmp(message, "CMD,2036,", 9) != 0){
		// invalid command
		return CMD_ERR;
	}
	// the message contains "CMD,2036," phrase, so it is ok to strtok() twice
	temp = strtok(message, ",");
	temp = strtok(NULL, ",");

	// from now on, temp may point to NULL
	temp = strtok(NULL, ",");
	if (temp == 0){
		return CMD_ERR;
	}

	else if (strcmp(temp, "CX") == 0){
		temp = strtok(NULL, ",");
		if (temp == 0) return CMD_ERR;	// invalid command

		if (strcmp(temp, "ON")) return CMD_CX_ON;
		if (strcmp(temp, "OFF")) return CMD_CX_OFF;

		return CMD_ERR_ARG;	// no such argument
	}
	else if (strcmp(temp, "ST") == 0){
		temp = strtok(NULL, ",");
		if (temp == 0) return CMD_ERR;

		if (strcmp(temp, "GPS")) return CMD_ST_GPS;

		return CMD_parseTime(temp, pArg);
	}
	else if (strcmp(temp, "SIM") == 0){
		temp = strtok(NULL, ",");
		if (temp == 0) return CMD_ERR;

		if (strcmp(temp, "ENABLE")) return CMD_SIM_ENABLE;
		if (strcmp(temp, "ACTIVATE")) return CMD_SIM_ACTIVATE;
		if (strcmp(temp, "DISABLE")) return CMD_SIM_DISABLE;

		return CMD_ERR_ARG;	// no such argument
	}
	else if (strcmp(temp, "SIMP") == 0){
		temp = strtok(NULL, ",");
		if (temp == 0) return CMD_ERR;

		return CMD_parseSIMP(temp, pArg);
	}
	else if (strcmp(temp, "CAL") == 0){
		return CMD_CAL;
	}
	else if (strcmp(temp, "BCN") == 0){
		temp = strtok(NULL, ",");
		if (temp == 0) return CMD_ERR;

		if (strcmp(temp, "ON")) return CMD_BCN_ON;
		if (strcmp(temp, "OFF")) return CMD_BCN_OFF;

		return CMD_ERR_ARG;	// no such argument
	}
	else if (strcmp(temp, "DEP") == 0){
		temp = strtok(NULL, ",");
		if (temp == 0) return CMD_ERR;

		if (strcmp(temp, "HS")) return CMD_DEP_HS;
		if (strcmp(temp, "PC")) return CMD_DEP_PC;

		return CMD_ERR_ARG;	// no such argument
	}
	else if (strcmp(temp, "REL") == 0){
		temp = strtok(NULL, ",");
		if (temp == 0) return CMD_ERR;

		if (strcmp(temp, "HS")) return CMD_REL_HS;

		return CMD_ERR_ARG;	// no such argument
	}
//	else if (strcmp(temp, "TEST") == 0){
//		temp = strtok(NULL, ",");
//		if (temp == 0) return CMD_ERR;
//
//		if (strcmp(temp, "")) return CMD_;
//		if (strcmp(temp, "")) return CMD_;
//		return CMD_ERR_ARG;	// no such argument
//	}
	else if (strcmp(temp, "INIT") == 0){
		return CMD_INIT;
	}
	else if (strcmp(temp, "RESET") == 0){
		return CMD_RESET;
	}
	else{
		// no such command
		return CMD_ERR_COM;
	}
}
CMD_CommandCaseTypeDef CMD_parseTime(uint8_t *message, int *pArg){
	// the function doesn't check NULL pointer error; check it before calling this function
	int i = 0;
	int field_num = 0;
	uint8_t time_var[3] = {0,};
	uint8_t ch;

	// i: 				01234567
	// message: 	hh:mm:ss

	while(1){
		ch = message[i];
		if ('0' <= ch && ch <= '9' && i != 2 && i != 5 && i <= 7){
			time_var[field_num] = time_var[field_num] * 10 + ch - '0';
		}
		else if (ch == ':' && ( i == 2 || i == 5 )){
			field_num++;
		}
		else if (ch == '\0' && i == 8){
			break;
		}
		else{
			*pArg = 0;
			return CMD_ERR_ARG;
		}
	}

	// check value
	if (time_var[0] > 24 || time_var[0] < 0){
		*pArg = 0;
		return CMD_ERR_ARG;
	}
	if (time_var[1] > 59 || time_var[1] < 0){
		*pArg = 0;
		return CMD_ERR_ARG;
	}
	if (time_var[2] > 59 || time_var[2] < 0){
		*pArg = 0;
		return CMD_ERR_ARG;
	}

	*pArg = time_var[0] * 10000 + time_var[1] * 100 + time_var[2];

	return CMD_ST_TIME;
}
CMD_CommandCaseTypeDef CMD_parseSIMP(uint8_t *message, int *pArg){
	uint8_t ch;
	int i = 0;
	int simp = 0;

	while(1){
		ch = message[i];
		if ('0' <= ch && ch <= '9'){
			simp = simp * 10 + ch - '0';
			i++;
		}
		else if (ch == '\0'){
			break;
		}
		else{
			*pArg = 0;
			return CMD_ERR_ARG;
		}
	}
	*pArg = simp;
	return CMD_SIMP;
}
void CMD_excuteCX_ON(void){
	uint32_t bkpdata;
  HAL_PWR_EnableBkUpAccess();
	bkpdata = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP0R);
	bkpdata |= ( 1U << 1 );
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP0R, bkpdata);
  HAL_PWR_DisableBkUpAccess();

	// ToDo: implement execution code
	logd("not implemented.\n");
	return;
}
void CMD_excuteCX_OFF(void){
	uint32_t bkpdata;
  HAL_PWR_EnableBkUpAccess();
	bkpdata = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP0R);
	bkpdata &= ~( 1U << 1 );
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP0R, bkpdata);
  HAL_PWR_DisableBkUpAccess();

	// ToDo: implement execution code
	logd("not implemented.\n");
	return;
}
void CMD_excuteST_TIME(int argument){
	RTC_TimeTypeDef sTime;
	uint8_t hours, minutes, seconds;

	seconds = argument % 100;
	argument /= 100;
	minutes = argument % 100;
	argument /= 100;
	hours = argument % 100;
	argument /= 100;

	sTime.Hours = hours;
	sTime.Minutes = minutes;
	sTime.Seconds = seconds;

	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	logi("RTC set to %d:%d:%d\n", hours, minutes, seconds);
	return;
}
void CMD_excuteST_GPS(void){
	sTime.Hours = gps_data.hours;
	sTime.Minutes = gps_data.minutes;
	sTime.Seconds = gps_data.seconds;
	logi("RTC set to %d:%d:%d\n", gps_data.hours, gps_data.minutes, gps_data.seconds);
	return;
}
void CMD_excuteSIM_ENABLE(void){
	// ToDo: implement execution code
	// if state is flight: deny
	// if state is simulation: do nothing
	// else: change state to SIM_ENABLED
	logd("not implemented.\n");
	return;
}
void CMD_excuteSIM_ACTIVATE(void){
	// ToDo: implement execution code
	// if state is SIM_ENABLED: change it to S_LAUNCH_WAIT
	// else: do nothing
	logd("not implemented.\n");
	return;
}
void CMD_excuteSIM_DISABLE(void){
	// ToDo: implement execution code
	// if state is one of simulation mode: change it to VEHICLE_RESET
	logd("not implemented.\n");
	return;
}
void CMD_excuteSIMP(int argument){
	// ToDo: implement execution code
	// if this is the first simp data after sim_activate: set simulated zero altitude calibration
	// else: change simulated pressure and change simulated altitude and raise simulated altitude update flag
	logd("not implemented.\n");
	return;
}
void CMD_excuteCAL(void){
	int ret = Calibrate();
	if ( ret ){
		logi("error occured. perform 8-figure for IMU calibration.\n");
	}
	else{
		logi("calibrated all sensors and saved the data.\n");
	}
	return;
}
void CMD_excuteBCN_ON(void){
	HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
	return;
}
void CMD_excuteBCN_OFF(void){
	HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_RESET);
	return;
}
void CMD_excuteDEP_HS(void){
	int deg = Servo_Read(&hservo1);
	//ToDo: implement state detection and state transfer

	// not in flight mode nor in simulation mode
	if (deg < 0){
		Error_Handler();
	}
	if (deg >= 90){
		Servo_Write(&hservo1, 0);
	}
	else{
		Servo_Write(&hservo1, 180);
	}
	return;
}
void CMD_excuteDEP_PC(void){
	int deg = Servo_Read(&hservo2);
	//ToDo: implement state detection and state transfer

	// not in flight mode nor in simulation mode
	if (deg < 0){
		Error_Handler();
	}
	if (deg >= 90){
		Servo_Write(&hservo2, 0);
	}
	else{
		Servo_Write(&hservo2, 180);
	}
	return;
}
void CMD_excuteREL_HS(void){
	int deg = Servo_Read(&hservo3);
	//ToDo: implement state detection and state transfer

	// not in flight mode nor in simulation mode
	if (deg < 0){
		Error_Handler();
	}
	if (deg >= 90){
		Servo_Write(&hservo3, 0);
	}
	else{
		Servo_Write(&hservo3, 180);
	}
	return;
}
void CMD_excuteINIT(void){
	// ToDo: implement execution code
	// if state is VEHICLE_RESET: change it to F_LAUNCH_WAIT
	logd("not implemented.\n");
	return;
}
void CMD_excuteRESET(void){
	// ToDo: implement state transfer to vehicle_reset state
	Backup();
	NVIC_SystemReset();
	return;
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
//  			Servo_Write(&hservo1, 180);	// deploy heat shield
  			vehicle_state = F_HS_DEPLOYED;
  		}
  		break;
  	case F_HS_DEPLOYED:
  		if (altitude < VEHICLE_PC_THRESHOLD){
  			Servo_Write(&hservo2, 180);	// release heat shield
  			Servo_Write(&hservo3, 180);	// deploy parachute
  			vehicle_state = F_PC_DEPLOYED;
  		}
  		break;
  	case F_PC_DEPLOYED:
  		if ((old_altitude - altitude) < VEHICLE_LAND_THRESHOLD &&\
  				(old_altitude - altitude) > -VEHICLE_LAND_THRESHOLD ){
  			// ToDo: implement CX OFF
  			HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
  			vehicle_state = F_LANDED;
  		}
  		break;
  	case F_LANDED:
  		HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
  		break;
  	case S_LAUNCH_WAIT:
  		if (s_altitude > VEHICLE_ASCENT_THRESHOLD) {
  			vehicle_state = S_ASCENT;
  		}
  		break;
  	case S_ASCENT:
  		if (s_max_altitude - s_altitude > VEHICLE_HS_THRESHOLD){
  			// PC deploy if needed
//  			Servo_Write(&hservo1, 180);	// deploy heat shield
  			vehicle_state = S_HS_DEPLOYED;
  		}
  		break;
  	case S_HS_DEPLOYED:
  		if (s_altitude < VEHICLE_PC_THRESHOLD){
  			Servo_Write(&hservo2, 180);	// release heat shield
  			Servo_Write(&hservo3, 180);	// deploy parachute
  			vehicle_state = S_PC_DEPLOYED;
  		}
  		break;
  	case S_PC_DEPLOYED:
  		if ((s_old_altitude - altitude) < VEHICLE_LAND_THRESHOLD &&\
  				(s_old_altitude - altitude) > -VEHICLE_LAND_THRESHOLD ){
  			// ToDo: implement CX OFF
  			HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
  			vehicle_state = S_LANDED;
  		}
  		break;
  	case S_LANDED:
			HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
			break;
  	default:
  		// VEHICLE_RESET, SIM_ENABLED
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

//  		cb_init();
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
	double tilt_x, tilt_y;
	uint16_t adc_0, adc_1;
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
	adc_0 = HAL_ADC_GetValue(&hadc1);

	// read ADC1 CH1
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 50);
	adc_1 = HAL_ADC_GetValue(&hadc1);

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

  double pressure_ratio = pressure * r_pressure_sea_level;
  altitude = (powf(pressure_ratio, ALTITUDE_POWER_COEFFICIENT) - 1) * ALTITUDE_PRODUCT_COEFFICIENT; // *100

  // calculate tilt angle
  double total_acc_r = 1 / sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
  tilt_x = asin( acc_x * total_acc_r );
  tilt_y = asin( acc_y * total_acc_r );

	// calculate battery voltage
  //battery_voltage = ADC1_CH0 / 4015 * 3.3 * 1.5 * 100;
  //battery_voltage = ADC1_CH0 * 0.123287671232877;
  battery_voltage = adc_0 / 4095.0 * 3.3 * 1.5;

	// calculate air speed
  air_speed = DP_calculateAirSpeedComp(adc_1, pressure / 100.f, temperature / 100.f);
  air_speed /= 100;

	// move data to sensor data container
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
  sensor_data_container.adc_0 = adc_0;
  sensor_data_container.adc_1 = adc_1;
  sensor_data_container.altitude = altitude;
  sensor_data_container.tilt_x = tilt_x;
  sensor_data_container.tilt_y = tilt_y;
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
	HAL_UART_Transmit(&huart3, tx_packet, sizeof(tx_packet), 50);
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
