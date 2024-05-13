/*
 * gps.h
 *
 *  Created on: Mar 26, 2024
 *      Author: SURFACE
 */

#ifndef INC_IH_DRIVERS_GPS_GPS_H_
#define INC_IH_DRIVERS_GPS_GPS_H_

#include "main.h"

#define GPS_MAX_LENGTH 82

typedef struct{
	uint8_t BCDhour;
	uint8_t BCDminute;
	uint8_t BCDsecond;
	int16_t altitude;
	int32_t latitude;
	int32_t longitude;
	int8_t satellites;
}GPS_DataTypeDef;

int GPS_NMEA_parseGGA(uint8_t *GPS_NMEA_Message, GPS_DataTypeDef* pGPS_Data);

#endif /* INC_IH_DRIVERS_GPS_GPS_H_ */
