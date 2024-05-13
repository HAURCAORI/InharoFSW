/*
 * sensor_init.h
 *
 *  Created on: May 8, 2024
 *      Author: sunny
 */

#ifndef INC_MODULE_INIT_SENSOR_H_
#define INC_MODULE_INIT_SENSOR_H_

#include "main.h"
#include "servo/servo.h"

typedef struct servo_HandleTypeDef Servo_HandleTypeDef;

void Init_BMP390(void);
void Init_BNO055(void);
void Init_SD(void);
void Init_Servo(Servo_HandleTypeDef* servo1, Servo_HandleTypeDef* servo2, Servo_HandleTypeDef* servo3);

#endif /* INC_MODULE_INIT_SENSOR_H_ */
