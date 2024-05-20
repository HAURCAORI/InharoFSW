/*
 * servo.h
 *
 *  Created on: Mar 20, 2024
 *      Author: SURFACE
 *
 *      clock: 		84MHz(internal clock)
 *      prescaler:	84
 *      ARR:		20000
 *      PWM generation CHx
 */


#ifndef SERVO_H_
#define SERVO_H_

#include "main.h"

#define SERVO_DEG_MIN	(0)
#define SERVO_DEG_MAX	(180)
#define SERVO_PW_MIN	(544)
#define SERVO_PW_MAX	(2400)

/* saturation / constraint */
#define _SERVO_SAT(n, min, max) (\
		(n) > (max) ? (max) :\
		(n) < (min) ? (min) :\
		(n))

/* deg to us, 2.4ms(max pw) at max deg, 0.544ms(min pw) at min deg*/
#define _SERVO_DEG2US(deg) (\
		(double)(SERVO_PW_MAX-SERVO_PW_MIN)/(SERVO_DEG_MAX-SERVO_DEG_MIN)*((deg)-SERVO_DEG_MIN)\
		+ (SERVO_PW_MIN))
#define _SERVO_US2DEG(us) (\
		(double)(SERVO_DEG_MAX-SERVO_DEG_MIN)/(SERVO_PW_MAX-SERVO_PW_MIN)*((us)-SERVO_PW_MIN)\
		+(SERVO_DEG_MIN))

typedef struct {
	// vars
	// timer channel using
	TIM_HandleTypeDef *htim;
	uint32_t channel;
}Servo_HandleTypeDef;

void Servo_Attach(Servo_HandleTypeDef *servo, TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef Servo_Write(Servo_HandleTypeDef *servo, int deg);
int Servo_Read(Servo_HandleTypeDef *servo);


#endif /* SERVO_H_ */
