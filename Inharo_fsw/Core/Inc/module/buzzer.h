/*
 * buzzer.h
 *
 *  Created on: May 8, 2024
 *      Author: sunny
 */

#ifndef __BUZZER_H__
#define __BUZZER_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "main.h"

void Buzzer_Once();
void Buzzer_OnceLong();
void Buzzer_Triple();
void Buzzer_On();
void Buzzer_Off();

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

