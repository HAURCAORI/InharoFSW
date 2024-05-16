#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "main.h"

#ifndef AUTO_GEN

// Definitions
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

// Prototypes
void MX_DMA_Init(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */
