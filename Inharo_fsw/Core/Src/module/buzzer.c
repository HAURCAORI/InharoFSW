/*
 * buzzer.c
 *
 *  Created on: May 8, 2024
 *      Author: sunny
 */

#ifndef SRC_MODULE_BUZZER_C_
#define SRC_MODULE_BUZZER_C_

#include "module/buzzer.h"

void Buzzer_Once() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	osDelay(100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

#endif /* SRC_MODULE_BUZZER_C_ */
