#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "main.h"

#ifndef AUTO_GEN

// Definitions
extern ADC_HandleTypeDef hadc1;

// Prototypes
void MX_ADC1_Init(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

