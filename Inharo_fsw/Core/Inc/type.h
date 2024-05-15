/*
 * fsw_type.h
 *
 *  Created on: May 8, 2024
 *      Author: sunny
 */

#ifndef INC_TYPE_H_
#define INC_TYPE_H_

#define DEBUG_CMD_SIZE 2
#define DEBUG_CMD_BUZZER 			0x7A62
#define DEBUG_CMD_CALIBRATION 0x6C63
#define DEBUG_CMD_PRESSURE 		0x7270


#define EVENT_RECEIVE_SUCCESS 1<<0
#define EVENT_RECEIVE_FAIL 		1<<1
#define EVENT_RECEIVE_USB 		1<<2
#define EVENT_RECEIVE_XBEE 		1<<3
#define EVENT_RECEIVE_GPS 		1<<4



#define USB_BUFFER_SIZE 8
#define PACKET_SIZE 		128
#define HEADER_SIZE 		8
#define CHECKSUM_SIZE 	1
#define TELEMETRY_PACKET_SIZE HEADER_SIZE + sizeof(Telemetry) + CHECKSUM_SIZE


#define FRAME_TYPE_TX 			0x01
#define FRAME_TYPE_RX 			0x81
#define FRAME_ADDRESS_HIGH 	0xCC
#define FRAME_ADDRESS_LOW 	0xCC

#define IH_UART1_MAX_LENGTH (80)
#define IH_UART1_HEADER ('$')
#define IH_UART1_HEADER1 ('G')
#define IH_UART1_HEADER2 ('G')
#define IH_UART1_HEADER3 ('A')
#define IH_UART1_TERMINATOR ('*')
#define IH_UART1_HEADER_OFFSET 	6


#define IH_UART3_HEADER 				0x7E
#define IH_UART3_HEADER_OFFSET 	3

typedef struct USB_Buffer_Type {
	uint8_t buffer[USB_BUFFER_SIZE];
	volatile uint16_t pos;
} USB_Buffer_Type;

typedef struct XBEE_Packet_Buffer {
	char isReceiving;
	uint8_t buffer[PACKET_SIZE];
	uint16_t length;
	uint16_t pos;
	uint16_t checksum;
} XBEE_Packet_Buffer;

typedef struct GPS_Packet_Buffer {
	char isReceiving;
	uint8_t buffer[PACKET_SIZE];
	uint8_t deviceID[2];
	uint8_t sentenceID[3];
	uint8_t chk[2];
	uint8_t checksum;
	uint16_t length;
	uint16_t pos;
} GPS_Packet_Buffer;

typedef struct XBEE_Packet {
	uint8_t data[PACKET_SIZE];
	uint16_t length;
} XBEE_Packet;

typedef struct GPS_Packet {
	uint8_t data[PACKET_SIZE];
	uint16_t length;
	uint8_t deviceID[2];
	uint8_t sentenceID[3];
} GPS_Packet;

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
	uint16_t subseconds;
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
	uint8_t GPS_time_hours;
	uint8_t GPS_time_minutes;
	uint8_t GPS_time_seconds;
	uint16_t GPS_time_subseconds;
	float GPS_altitude;
	float GPS_latitude;
	float GPS_longitude;
	uint8_t GPS_sats;
	float tilt_x;
	float tilt_y;
	float rot_z;
	uint16_t cmd_echo;
} Telemetry;
#pragma pack(pop)

#endif /* INC_TYPE_H_ */
