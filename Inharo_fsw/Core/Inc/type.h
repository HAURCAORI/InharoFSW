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


#define EVENT_RECEIVE_SUCCESS 1<<0
#define EVENT_RECEIVE_FAIL 1<<1
#define EVENT_RECEIVE_USB 1<<2
#define EVENT_RECEIVE_XBEE 1<<3



#define USB_BUFFER_SIZE 8
#define XBEE_BUFFER_SIZE 128
#define RECEIVE_BUFFER_SIZE 128


typedef struct USB_Buffer_Type {
	uint8_t buffer[USB_BUFFER_SIZE];
	volatile uint16_t pos;
} USB_Buffer_Type;

typedef struct XBEE_Buffer_Type {
	char isReceiving;
	char isValid;
	uint8_t buffer[XBEE_BUFFER_SIZE];
	uint16_t length;
	uint16_t pos;
	uint16_t checksum;
} XBEE_Buffer_Type;

typedef struct Sensor_Data {
	float pressure;
	float temperature;
} Sensor_Data;



#endif /* INC_TYPE_H_ */
