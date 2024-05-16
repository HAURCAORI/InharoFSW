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

typedef struct GPS_DataTypeDef{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	float altitude;
	double latitude;
	double longitude;
	uint8_t satellites;
}GPS_DataTypeDef;

int GPS_NMEA_parseGGA(uint8_t *GPS_NMEA_Message, GPS_DataTypeDef* pGPS_Data);

#endif /* INC_IH_DRIVERS_GPS_GPS_H_ */
