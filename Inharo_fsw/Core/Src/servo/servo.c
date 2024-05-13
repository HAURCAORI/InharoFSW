#include "servo/servo.h"

void Servo_Attach(Servo_HandleTypeDef *servo, TIM_HandleTypeDef *htim, uint32_t channel){
	servo->htim 	= htim;
	servo->channel 	= channel;
}


HAL_StatusTypeDef Servo_Write(Servo_HandleTypeDef *servo, int deg){
	uint32_t us = 0;
	switch (servo->channel){
	case TIM_CHANNEL_1:
		deg = _SERVO_SAT(deg, SERVO_DEG_MIN, SERVO_DEG_MAX);
		us = _SERVO_DEG2US(deg);
		us = _SERVO_SAT(us, SERVO_PW_MIN, SERVO_PW_MAX);
		servo->htim->Instance->CCR1 = us;
		break;
	case TIM_CHANNEL_2:
		deg = _SERVO_SAT(deg, SERVO_DEG_MIN, SERVO_DEG_MAX);
		us = _SERVO_DEG2US(deg);
		us = _SERVO_SAT(us, SERVO_PW_MIN, SERVO_PW_MAX);
		servo->htim->Instance->CCR2 = us;
		break;
	case TIM_CHANNEL_3:
		deg = _SERVO_SAT(deg, SERVO_DEG_MIN, SERVO_DEG_MAX);
		us = _SERVO_DEG2US(deg);
		us = _SERVO_SAT(us, SERVO_PW_MIN, SERVO_PW_MAX);
		servo->htim->Instance->CCR3 = us;
		break;
	case TIM_CHANNEL_4:
		deg = _SERVO_SAT(deg, SERVO_DEG_MIN, SERVO_DEG_MAX);
		us = _SERVO_DEG2US(deg);
		us = _SERVO_SAT(us, SERVO_PW_MIN, SERVO_PW_MAX);
		servo->htim->Instance->CCR4 = us;
		break;
	default:
		return HAL_ERROR;
	}
	return HAL_OK;
}
