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
#define PACKET_SIZE 128
#define HEADER_SIZE 8
#define CHECKSUM_SIZE 1
#define TELEMETRY_PACKET_SIZE HEADER_SIZE + sizeof(Telemetry) + CHECKSUM_SIZE


#define FRAME_TX 1
#define FRAME_ADDRESS_HIGH 0xCC
#define FRAME_ADDRESS_LOW 0xCC

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

#pragma pack(push, 1)
typedef struct Telemetry {
	uint16_t team_id;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint16_t packet_count;
	uint8_t mode;
	uint16_t state;
	float altitude;
	float air_speed;
	uint8_t heat_shield;
	uint8_t parachute;
	float temperature;
	float voltage;
	float pressure;
	float GPS_time;
	float GPS_altitude;
	float GPS_latitude;
	float GPS_longitude;
	uint8_t GPS_sats;
	float tilt_x;
	float tilt_y;
	float rot_z;
	uint8_t cmd_echo[10];
} Telemetry;
#pragma pack(pop)

#endif /* INC_TYPE_H_ */
