/*
 * bmp390_stm32.h
 *
 *  Created on: Mar 26, 2024
 *      Author: SURFACE
 */

#ifndef INC_IH_DRIVERS_BMP390_BMP390_STM32_H_
#define INC_IH_DRIVERS_BMP390_BMP390_STM32_H_

/* for practical use, one must modify BMP390_Init() function !!! */

#include <module/bmp390/bmp3.h>
#include "main.h"

#define BMP390_I2C_TIMEOUT		100

BMP3_INTF_RET_TYPE BMP390_Read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr);
BMP3_INTF_RET_TYPE BMP390_Write(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, void *intf_ptr);
void BMP390_DelayUs(uint32_t period, void *intf_ptr);
void BMP390_Init(void);
HAL_StatusTypeDef BMP390_GetValue( int64_t *pTemperature, uint64_t *pPressure, uint32_t Timeout );
void BMP390_AssignI2C(I2C_HandleTypeDef *phi2c);

#endif /* INC_IH_DRIVERS_BMP390_BMP390_STM32_H_ */
