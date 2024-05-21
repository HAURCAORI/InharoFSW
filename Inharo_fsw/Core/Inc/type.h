/*
 * fsw_type.h
 *
 *  Created on: May 8, 2024
 *      Author: sunny
 */

#ifndef INC_TYPE_H_
#define INC_TYPE_H_

#define PI 3.14159265


#define DEBUG_CMD_SIZE 2
#define DEBUG_CMD_BUZZER 			0x7A62
#define DEBUG_CMD_CALIBRATION 0x6C63
#define DEBUG_CMD_PRESSURE 		0x7270
#define DEBUG_CMD_SERVO_180		0x6F73
#define DEBUG_CMD_SERVO_0			0x6673

#define EVENT_RECEIVE_SUCCESS 1<<0
#define EVENT_RECEIVE_FAIL 		1<<1
#define EVENT_RECEIVE_USB 		1<<2
#define EVENT_RECEIVE_XBEE 		1<<3
#define EVENT_RECEIVE_GPS 		1<<4
#define EVENT_RECEIVE_ACK 		1<<5
#define EVENT_RECEIVE_PUSH		1<<6


#define USB_BUFFER_SIZE 8
#define PACKET_SIZE 		128
#define HEADER_SIZE 		8
#define CHECKSUM_SIZE 	1
#define TELEMETRY_PACKET_SIZE HEADER_SIZE + sizeof(Telemetry) + CHECKSUM_SIZE


#define FRAME_TYPE_TX 			0x01
#define FRAME_TYPE_RX 			0x81
#define FRAME_ADDRESS_HIGH 	0xCC
#define FRAME_ADDRESS_LOW 	0xCC

#define GPS_SENTENCE_GGA 		0x474741

#define RX_HEADER_CMD				0x444D43   	//'CMD'
#define RX_HEADER_ACK				0x4B4341   	//'ACK'
#define RX_HEADER_TEAM_ID 	0x36333032	//'2036'
#define RX_CMD_CX						0x5843			//'CX
#define RX_CMD_ST						0x5453			//'ST'
#define RX_CMD_CAL					0x4C4143		//'CAL'
#define RX_CMD_SIM					0x4D4953		//'SIM'
#define RX_CMD_BCN					0x4E4342		//'BCN'
#define RX_CMD_REL					0x4C4552		//'REL'
#define RX_CMD_DEP					0x504544		//'DEP'
#define RX_CMD_RESET				0x534552		//'RES'
#define RX_CMD_INIT					0x54494E49	//'INIT'


#define IH_CX_OFF						0x00
#define IH_CX_ON						0xFF

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
	float GPS_altitude;
	float GPS_latitude;
	float GPS_longitude;
	uint8_t GPS_sats;
	float tilt_x;
	float tilt_y;
	float rot_z;
	uint16_t cmd_echo;
} Telemetry;

typedef enum VehicleStateTypeDef{
	VEHICLE_RESET = 	0x00,
	F_LAUNCH_WAIT = 	0x01,
	F_ASCENT = 				0x02,
	F_HS_DEPLOYED = 	0x03,
	F_PC_DEPLOYED = 	0x04,
	F_LANDED = 				0x05,

	SIM_ENABLED = 		0x10,
	S_LAUNCH_WAIT = 	0x11,
	S_ASCENT = 				0x12,
	S_HS_DEPLOYED = 	0x13,
	S_PC_DEPLOYED = 	0x14,
	S_LANDED = 				0x15
}VehicleStateTypeDef;

typedef enum CommandEcho{
  ECHO_NONE					= 0X00,
	ECHO_CX_ON 				= 0x01,
	ECHO_CX_OFF 			= 0x02,
	ECHO_ST_UTC 			= 0x03,
	ECHO_ST_GPS				= 0x04,
	ECHO_SIM_ENABLE 	= 0x05,
	ECHO_SIM_ACTIVATE	= 0x06,
	ECHO_SIM_DISABLE	= 0x07,
	ECHO_CAL					= 0x08,
	ECHO_BCN_ON				= 0x09,
	ECHO_BCN_OFF			= 0x0A,
	ECHO_REL_HS				= 0x0B,
	ECHO_DEP_PC				= 0x0C,
	ECHO_INIT					= 0xFE,
	ECHO_RESET				= 0xFF
} CommandEcho;

#pragma pack(pop)

#endif /* INC_TYPE_H_ */
