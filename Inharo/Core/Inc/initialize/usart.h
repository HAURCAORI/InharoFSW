#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "main.h"

#ifndef AUTO_GEN

// Definitions
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Prototypes
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

