/*
 * fsw_type.h
 *
 *  Created on: May 8, 2024
 *      Author: sunny
 */

#ifndef INC_TYPE_H_
#define INC_TYPE_H_

#define DEBUG_CMD_SIZE 2
#define DEBUG_CMD_BUZZER 0x7A62
#define DEBUG_CMD_CALIBRATION 0x6C63
#define DEBUG_CMD_PRESSURE 0x7270


#define RECEIVED_USB 1<<0

#define USB_BUFFER_SIZE 8

typedef struct USB_Buffer_Type {
	uint8_t buffer[USB_BUFFER_SIZE];
	volatile uint16_t pos;
} USB_Buffer_Type;


typedef struct Sensor_Data {
	float pressure;
	float temperature;
} Sensor_Data;



#endif /* INC_TYPE_H_ */
