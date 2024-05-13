/*
 * init_sensor.c
 *
 *  Created on: May 8, 2024
 *      Author: sunny
 */

#include <module/init_sensor.h>



void Init_BMP390(void){
	HAL_StatusTypeDef res1, res2, res3, res4, res5, res6, res7;

	BMP390_AssignI2C(&hi2c1);

	/* Check whether the sensor is ready */
	/*	if ( HAL_I2C_IsDeviceReady(bmp390_phi2c, BMP390_ADDRESS, 10, 10\) != HAL_OK ) Error_Handler();*/

	/* Soft reset */
	res1 = BMP390_SoftReset(1000);
	if ( BMP390_SoftReset(1000) != HAL_OK ) Error_Handler();

	/* Set sensor configuration buffers  */
	uint8_t bmp390PWR = BMP390_PWR_ALL_NORMAL;
	uint8_t bmp390OSR = BMP390_OSR_PX08;
	uint8_t bmp390ODR = BMP390_ODR_50;
	uint8_t bmp390CFG = BMP390_CFG_IIR1;

	/* Initial setting */
	if ( (res2 = BMP390_Write(BMP390_REG_OSR, &bmp390OSR, sizeof(bmp390OSR), 10)) != HAL_OK ) Error_Handler();
	if ( (res3 = BMP390_Write(BMP390_REG_ODR, &bmp390ODR, sizeof(bmp390ODR), 10)) != HAL_OK ) Error_Handler();
	if ( (res4 = BMP390_Write(BMP390_REG_CFG, &bmp390CFG, sizeof(bmp390CFG), 10)) != HAL_OK ) Error_Handler();

	/* Don't know why, but it seems like the PWR_CTRL register need to be set\
	 *  twice */
	if ( (res5 = BMP390_Write(BMP390_REG_PWR, &bmp390PWR, sizeof(bmp390PWR), 10)) != HAL_OK ) Error_Handler();
	if ( (res6 = BMP390_Write(BMP390_REG_PWR, &bmp390PWR, sizeof(bmp390PWR), 10)) != HAL_OK ) Error_Handler();

	osDelay(1000);

	if ( (res7 = BMP390_ReadCalibration()) != HAL_OK) Error_Handler();

	return;
}

void Init_BNO055(void){
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();
}

void Init_SD(void){
	//SD_Assign(&hspi2);
	//retUSER = f_mount(&USERFatFS, USERPath, 1);
	//if ( retUSER != FR_OK ) Error_Handler();
}

void Init_Servo(Servo_HandleTypeDef* servo1, Servo_HandleTypeDef* servo2, Servo_HandleTypeDef* servo3){
	Servo_Attach(servo1, &htim3, TIM_CHANNEL_1);
	Servo_Attach(servo2, &htim3, TIM_CHANNEL_2);
	Servo_Attach(servo3, &htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
}
