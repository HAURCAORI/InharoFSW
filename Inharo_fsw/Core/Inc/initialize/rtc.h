#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "main.h"

#ifndef AUTO_GEN

// Definitions
extern RTC_HandleTypeDef hrtc;

// Prototypes
void MX_RTC_Init(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */

